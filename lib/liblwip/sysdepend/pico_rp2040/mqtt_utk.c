/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_init() once before its
 *	poll loop and mqtt_utk_start() on every pass while the link is up.
 *	mqtt.c's own keepalive/ping timers, the application poll chain and
 *	the reconnect back-off all ride the same sys_check_timeouts() polling
 *	that task already does for DHCP and TCP -- nothing extra runs on any
 *	other task or core.
 *
 *	PicoCar changes from the template demo (Week 6 Section 6.5, A14;
 *	agreed with the team and the instructor, recorded as IF-11 in TEAM.md):
 *	  - the 5 s "hb" demo publish is gone: the application publishes a
 *	    retained status message at 1 Hz from mqtt_utk_app_poll();
 *	  - keep-alive 60 s -> 10 s, and a Last Will from the application;
 *	  - fixed 3 s retry -> 1, 2, 4, 8, 16 s exponential back-off, reset on
 *	    every accepted connection;
 *	  - subscribes to the application's command topic only, not to the
 *	    topic it publishes on;
 *	  - received payloads are passed on byte for byte (the demo replaced
 *	    non-printable bytes with '.'), and oversized ones are dropped and
 *	    counted instead of being truncated;
 *	  - no tm_printf per message: on the dual-core USB build tm_printf
 *	    masks interrupts on core 0, whose tick drives every cyclic handler.
 *	    Only connection changes are printed;
 *	  - mqtt_publish() results are returned to the caller, and failures
 *	    reported later by lwIP are counted.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S		10
#define MQTT_UTK_BACKOFF_MIN_MS		1000u
#define MQTT_UTK_BACKOFF_MAX_MS		16000u
#define MQTT_UTK_POLL_MS		20u

/* Inbound message staging. A publish arrives as one topic callback followed
 * by one or more data callbacks, so the topic has to be kept until the
 * fragment carrying MQTT_DATA_FLAG_LAST shows up. Commands are well under
 * 128 bytes; anything longer is dropped whole and counted. */
#define MQTT_UTK_RX_TOPIC_MAX		64
#define MQTT_UTK_RX_DATA_MAX		128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;

/* 1 while a connect attempt is running or a back-off retry is pending;
 * mqtt_utk_start() does nothing then. */
static u8_t		busy;
static u32_t		backoff_ms = MQTT_UTK_BACKOFF_MIN_MS;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static u8_t		rx_data[MQTT_UTK_RX_DATA_MAX + 1];
static u16_t		rx_len;
static u8_t		rx_too_long;

static u32_t		pub_failures;
static u32_t		rx_dropped;

/* ------------------------------------------------------------------------ */
/* Weak application hooks: a build without the PicoCar application links. */

__attribute__((weak)) const T_MQTT_UTK_CFG *mqtt_utk_app_config(void)
{
	return NULL;
}

__attribute__((weak)) void mqtt_utk_app_connected(void)
{
}

__attribute__((weak)) void mqtt_utk_app_disconnected(void)
{
}

__attribute__((weak)) void mqtt_utk_app_poll(void)
{
}

__attribute__((weak)) void mqtt_utk_app_rx(const char *topic,
					   const uint8_t *data, uint16_t len)
{
	(void)topic;
	(void)data;
	(void)len;
}

/* ------------------------------------------------------------------------ */

static void poll_timeout(void *arg)
{
	(void)arg;
	mqtt_utk_app_poll();
	sys_timeout(MQTT_UTK_POLL_MS, poll_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	busy = 0;	/* the poll loop calls mqtt_utk_start() while the link is up */
}

static void schedule_retry(void)
{
	tm_printf((UB*)"mqtt: retry in %u s\n", (unsigned int)(backoff_ms / 1000u));
	busy = 1;
	sys_untimeout(retry_timeout, NULL);
	sys_timeout(backoff_ms, retry_timeout, NULL);

	backoff_ms *= 2u;
	if(backoff_ms > MQTT_UTK_BACKOFF_MAX_MS) {
		backoff_ms = MQTT_UTK_BACKOFF_MAX_MS;
	}
}

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_too_long = (tot_len > MQTT_UTK_RX_DATA_MAX)? 1: 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len && rx_len < MQTT_UTK_RX_DATA_MAX; i++) {
		rx_data[rx_len++] = data[i];
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = 0;	/* convenience terminator; length is authoritative */
		if(rx_too_long) {
			rx_dropped++;
		} else {
			mqtt_utk_app_rx(rx_topic, rx_data, rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		pub_failures++;
	}
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	const T_MQTT_UTK_CFG	*cfg = mqtt_utk_app_config();

	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		busy = 0;
		backoff_ms = MQTT_UTK_BACKOFF_MIN_MS;
		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Callbacks must be registered before subscribing, or the
		 * first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);
		if(cfg != NULL && cfg->sub_topic != NULL) {
			err_t err = mqtt_subscribe(client, cfg->sub_topic,
						   cfg->sub_qos,
						   subscribe_done_cb, NULL);
			if(err != ERR_OK) {
				tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
					  (int)err);
			}
		}
		mqtt_utk_app_connected();
	} else {
		/* Refused, timed out (keep-alive lost) or disconnected. lwIP has
		 * already closed the client, so it can be connected again after
		 * the back-off. */
		tm_printf((UB*)"mqtt: connection status %d\n", (int)status);
		mqtt_utk_app_disconnected();
		schedule_retry();
	}
}

void mqtt_utk_init(void)
{
	sys_untimeout(poll_timeout, NULL);
	sys_timeout(MQTT_UTK_POLL_MS, poll_timeout, NULL);
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	const T_MQTT_UTK_CFG			*cfg;
	err_t					err;

	if(busy || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			schedule_retry();
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		schedule_retry();
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	cfg = mqtt_utk_app_config();
	if(cfg != NULL && cfg->will_topic != NULL && cfg->will_msg != NULL) {
		client_info.will_topic = cfg->will_topic;
		client_info.will_msg = cfg->will_msg;
		client_info.will_qos = cfg->will_qos;
		client_info.will_retain = cfg->will_retain;
	}

	busy = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		schedule_retry();
	}
}

uint8_t mqtt_utk_is_connected(void)
{
	return (client != NULL && mqtt_client_is_connected(client))? 1u: 0u;
}

int32_t mqtt_utk_publish(const char *topic, const uint8_t *data,
			 uint16_t len, uint8_t qos, uint8_t retain)
{
	if(client == NULL || !mqtt_client_is_connected(client)) {
		return MQTT_UTK_E_OFFLINE;
	}
	return (int32_t)mqtt_publish(client, topic, data, len, qos, retain,
				     publish_done_cb, NULL);
}

uint32_t mqtt_utk_pub_failures(void)
{
	return pub_failures;
}

uint32_t mqtt_utk_rx_dropped(void)
{
	return rx_dropped;
}

/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	PicoCar extension (agreed with the team and the instructor, Week 6
 *	A14 / TEAM.md IF-11).  Everything here runs on the WiFi service task
 *	cyw43_utk_task (core 0, priority 4) -- the only task allowed to call
 *	lwIP.  The application side lives in app_program/subsystem1/
 *	comms_service.c and is reached through the hook functions at the
 *	bottom of this file, which mqtt_utk.c calls from that same task.
 *
 *	Only <stdint.h> types are used, so this header is safe both in the
 *	pico-sdk/lwIP namespace (cyw43_utk.c, mqtt_utk.c) and next to
 *	tk/tkernel.h in application code.
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

#include <stdint.h>

/* Returned by mqtt_utk_publish() when there is no broker connection.
   Other negative values are lwIP err_t codes (ERR_MEM = -1 when the
   output ring or the request slots are full). */
#define MQTT_UTK_E_OFFLINE	(-100)

typedef struct {
	const char	*sub_topic;	/* NULL = subscribe to nothing        */
	uint8_t		sub_qos;
	const char	*will_topic;	/* NULL = no Last Will                */
	const char	*will_msg;	/* text; its strlen() is used         */
	uint8_t		will_qos;
	uint8_t		will_retain;
} T_MQTT_UTK_CFG;

/*
 * Call once from cyw43_utk_task() before its poll loop.  Starts the
 * application poll chain (mqtt_utk_app_poll() every MQTT_UTK_POLL_MS), so
 * the application keeps draining its queues while the link is down.
 */
void mqtt_utk_init(void);

/*
 * Called by cyw43_utk_task() on every poll pass while the link is up.
 * Starts a connection when none is running and no back-off is pending;
 * otherwise it does nothing.
 */
void mqtt_utk_start(void);

/* 1 while connected to the broker. */
uint8_t mqtt_utk_is_connected(void);

/*
 * Publish one message.  Never blocks.  Returns 0 when lwIP queued it,
 * MQTT_UTK_E_OFFLINE, or a negative lwIP err_t.  The caller owns retries.
 */
int32_t mqtt_utk_publish(const char *topic, const uint8_t *data,
			 uint16_t len, uint8_t qos, uint8_t retain);

/* QoS 1/2 publishes that lwIP later reported as failed (e.g. timeout). */
uint32_t mqtt_utk_pub_failures(void);

/* Received messages dropped because they exceeded MQTT_UTK_RX_DATA_MAX. */
uint32_t mqtt_utk_rx_dropped(void);

/* ---------------------------------------------------------------------
 * Application hooks.  mqtt_utk.c provides weak empty defaults, so a build
 * without the application still links.  All are called on the WiFi
 * service task and must not block.
 * --------------------------------------------------------------------- */
const T_MQTT_UTK_CFG *mqtt_utk_app_config(void);	/* NULL = defaults */
void mqtt_utk_app_connected(void);
void mqtt_utk_app_disconnected(void);
void mqtt_utk_app_poll(void);
void mqtt_utk_app_rx(const char *topic, const uint8_t *data, uint16_t len);

#endif /* MQTT_UTK_H */

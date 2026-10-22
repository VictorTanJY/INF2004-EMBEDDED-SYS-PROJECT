/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip). */
#define MQTT_BROKER_IP   "192.168.1.10"
#define MQTT_BROKER_PORT 1883
/* PicoCar: the client ID must be unique on the broker -- two clients with
 * the same ID disconnect each other.  The topics are fixed in
 * app_program/subsystem1/subsystem1.h (picocar/is23/...); MQTT_TOPIC is no
 * longer used by the client and is kept only for the template's history. */
#define MQTT_CLIENT_ID   "picocar-is23"
#define MQTT_TOPIC       "mtk3smp-pico/heartbeat"

#endif /* WIFI_SECRETS_H */

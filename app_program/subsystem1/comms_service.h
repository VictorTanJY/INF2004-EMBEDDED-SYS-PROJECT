/** @file comms_service.h
 *
 *  @brief CommsTask application side: the MQTT client hooks declared in
 * mqtt_utk.h (mqtt_utk_app_config, _connected, _disconnected, _poll, _rx) are
 * defined in comms_service.c.
 */

#ifndef COMMS_SERVICE_H
#define COMMS_SERVICE_H

#include "../app.h" /* TM_MQTT */

#if TM_MQTT
#include "mqtt_utk.h"
#endif

/* No other interface: mqtt_utk.c calls the hooks on the WiFi service task. */

#endif /* COMMS_SERVICE_H */

/*** end of file ***/

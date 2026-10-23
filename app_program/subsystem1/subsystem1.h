/*
 *  subsystem1.h
 *  Subsystem 1 -- WiFi communication, command and telemetry (CommsTask,
 *  i.e. the template's WiFi/MQTT service) and the buttons (HmiTask).
 *  Owner: Buddy 1 (Victor, 2503553).
 *
 *  PUBLIC interface of subsystem 1.  Week 6 design review: Sections 3,
 *  4 and 6.5.
 *
 *  IF-12  mbf_telem: telemetry records from any task.  Post them with
 *         comms_telem_post(); it never blocks (TMO_POL) and counts a drop
 *         when the buffer is full (NFR10: telemetry never blocks a
 *         control task).
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include <stdint.h>
#include <stdbool.h>
#include "../app.h"
#include "comms_proto.h"

/* Task placement (TEAM.md section 4).  CommsTask is the template's
   cyw43_utk_task (priority 4, core 0), created by the kernel. */
#define S1_PRI_HMI              11
#define S1_CORE_HMI             APP_CORE0

/* Buttons (TEAM.md section 2).  Robo Pico on-board buttons, internal
   pull-up enabled, pressed = LOW.  S1-M01 confirms the level on the board. */
#define S1_PIN_BTN_START        20u
#define S1_PIN_BTN_MODE         21u
#define S1_BTN_PRESSED_LEVEL    0u

#define HMI_PERIOD_MS           20u     /* sample period = debounce time */
#define HMI_LONG_PRESS_MS       2000u   /* BTN_MODE long press = fault reset */

/* mbf_telem: 1 KB = 64 records of 12 B (+4 B header each). */
#define S1_MBF_TELEM_BYTES      1024u

IMPORT ID g_mbf_telem;          /* mbf_telem in the design documents */

/* Post one record (tasks only, not ISRs).  true = queued, false = dropped. */
IMPORT bool comms_telem_post(uint8_t kind, uint8_t src, uint16_t code, int32_t data);

/* Records dropped because mbf_telem was full. */
IMPORT uint32_t comms_telem_drops(void);

/* MQTT topics (TEAM.md IF-11).  COMMS_TOPIC_BASE may be overridden with
   -DCOMMS_TOPIC_BASE=... on the make command line, e.g. for a second robot. */
#ifndef COMMS_TOPIC_BASE
#define COMMS_TOPIC_BASE        "picocar/is23"
#endif
#define COMMS_TOPIC_TELEMETRY   COMMS_TOPIC_BASE "/telemetry"
#define COMMS_TOPIC_EVENT       COMMS_TOPIC_BASE "/event"
#define COMMS_TOPIC_STATUS      COMMS_TOPIC_BASE "/status"
#define COMMS_TOPIC_CMD         COMMS_TOPIC_BASE "/cmd"
#define COMMS_TOPIC_ACK         COMMS_TOPIC_BASE "/ack"

#define COMMS_TELEM_PERIOD_MS   200u    /* 5 Hz (NFR9)  */
#define COMMS_STATUS_PERIOD_MS  1000u   /* 1 Hz heartbeat */

/* Private to subsystem 1 */
IMPORT void hmi_task(INT stacd, void * p_exinf);

#endif /* SUBSYSTEM1_H */

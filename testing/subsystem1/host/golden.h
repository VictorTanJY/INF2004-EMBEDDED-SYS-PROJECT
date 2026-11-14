/*
 *  golden.h
 *  Golden frames (hex) for the S1 host tests.  Each was produced by the C
 *  encoder (comms_proto.c) and decoded independently by the laptop codec
 *  (dashboard/picocar_codec.py) back to the inputs in test_s1.c;
 *  test_codec.py (S1-H21) repeats that decode on every run.
 */
#define GOLD_TELEMETRY      "210c540100a3af7de803de032b028803c002140e0b334cfcffff03"
#define GOLD_EVENT_BARCODE  "22054d00000c9110"
#define GOLD_EVENT_LOG      "22084d00002f3a9e158d04"
#define GOLD_STATUS         "233d00740300"
#define GOLD_STATUS_MAP     "230100780037880c01"
#define GOLD_ACK_OK         "245500"
#define GOLD_ACK_NO_ID      "240004"
#define GOLD_CMD_SET        "2509057d40"
#define GOLD_CMD_START      "250701"
#define GOLD_CMD_MAP        "25055e203204"

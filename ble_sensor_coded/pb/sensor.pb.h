/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.4 at Sun Dec  1 13:49:29 2019. */

#ifndef PB_SENSOR_PB_H_INCLUDED
#define PB_SENSOR_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _CentralMessage {
    int32_t rssi;
    pb_byte_t RemoteId[6];
/* @@protoc_insertion_point(struct:CentralMessage) */
} CentralMessage;

typedef PB_BYTES_ARRAY_T(6) Reading_Id_t;
typedef struct _Reading {
    Reading_Id_t Id;
    int32_t Temperature;
    int32_t Humidity;
    int32_t Battery;
    float Current;
/* @@protoc_insertion_point(struct:Reading) */
} Reading;

typedef struct _SensorMessage {
    pb_size_t Readings_count;
    Reading Readings[6];
    bool has_ProxyCentral;
    CentralMessage ProxyCentral;
    int32_t rssi;
    int32_t BatteryVoltage;
/* @@protoc_insertion_point(struct:SensorMessage) */
} SensorMessage;

/* Default values for struct fields */

/* Initializer values for message structs */
#define CentralMessage_init_default              {0, {0}}
#define SensorMessage_init_default               {0, {Reading_init_default, Reading_init_default, Reading_init_default, Reading_init_default, Reading_init_default, Reading_init_default}, false, CentralMessage_init_default, 0, 0}
#define Reading_init_default                     {{0, {0}}, 0, 0, 0, 0}
#define CentralMessage_init_zero                 {0, {0}}
#define SensorMessage_init_zero                  {0, {Reading_init_zero, Reading_init_zero, Reading_init_zero, Reading_init_zero, Reading_init_zero, Reading_init_zero}, false, CentralMessage_init_zero, 0, 0}
#define Reading_init_zero                        {{0, {0}}, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define CentralMessage_rssi_tag                  1
#define CentralMessage_RemoteId_tag              2
#define Reading_Id_tag                           1
#define Reading_Temperature_tag                  2
#define Reading_Humidity_tag                     3
#define Reading_Battery_tag                      4
#define Reading_Current_tag                      5
#define SensorMessage_Readings_tag               1
#define SensorMessage_ProxyCentral_tag           2
#define SensorMessage_rssi_tag                   3
#define SensorMessage_BatteryVoltage_tag         4

/* Struct field encoding specification for nanopb */
extern const pb_field_t CentralMessage_fields[3];
extern const pb_field_t SensorMessage_fields[5];
extern const pb_field_t Reading_fields[6];

/* Maximum encoded size of messages (where known) */
#define CentralMessage_size                      14
#define SensorMessage_size                       291
#define Reading_size                             41

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define SENSOR_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif

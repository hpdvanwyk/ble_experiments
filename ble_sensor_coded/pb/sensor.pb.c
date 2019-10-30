/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.4 at Wed Oct 30 22:58:45 2019. */

#include "sensor.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t CentralMessage_fields[3] = {
    PB_FIELD(  1, SINT32  , SINGULAR, STATIC  , FIRST, CentralMessage, rssi, rssi, 0),
    PB_FIELD(  2, FIXED_LENGTH_BYTES, SINGULAR, STATIC  , OTHER, CentralMessage, RemoteId, rssi, 0),
    PB_LAST_FIELD
};

const pb_field_t SensorMessage_fields[4] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, SensorMessage, Readings, Readings, &Reading_fields),
    PB_FIELD(  2, MESSAGE , OPTIONAL, STATIC  , OTHER, SensorMessage, ProxyCentral, Readings, &CentralMessage_fields),
    PB_FIELD(  3, SINT32  , SINGULAR, STATIC  , OTHER, SensorMessage, rssi, ProxyCentral, 0),
    PB_LAST_FIELD
};

const pb_field_t Reading_fields[5] = {
    PB_FIELD(  1, BYTES   , SINGULAR, STATIC  , FIRST, Reading, Id, Id, 0),
    PB_FIELD(  2, SINT32  , SINGULAR, STATIC  , OTHER, Reading, Temperature, Id, 0),
    PB_FIELD(  3, INT32   , SINGULAR, STATIC  , OTHER, Reading, Humidity, Temperature, 0),
    PB_FIELD(  4, INT32   , SINGULAR, STATIC  , OTHER, Reading, Battery, Humidity, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(SensorMessage, Readings[0]) < 65536 && pb_membersize(SensorMessage, ProxyCentral) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_CentralMessage_SensorMessage_Reading)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_16BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in the default
 * 8 bit descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(SensorMessage, Readings[0]) < 256 && pb_membersize(SensorMessage, ProxyCentral) < 256), YOU_MUST_DEFINE_PB_FIELD_16BIT_FOR_MESSAGES_CentralMessage_SensorMessage_Reading)
#endif


/* @@protoc_insertion_point(eof) */

/*

     ___        _____ _ ____
    /   | _____/ ___/(_) __ \
   / /| |/ ___/\__ \/ / /_/ / Tech Co., LTD
  / ___ / /__ ___/ / / ____/
 /_/  |_\___//____/_/_/
    (C)2018 AcSip

Description: GPS receiver driver for SONY CXD5603
*/

#ifndef __GPS_DRIVER_H__
#define __GPS_DRIVER_H__

#include "board.h"
#include "uart-board.h"

/*!
 * Enable GPS active low power control mode or Not
 */
#define GPS_ActiveLowPowerControlMode            0        // 1:Enable, 0:Disable
#define GPS_ActiveLowPowerMode_DUTYCYCLE         60000    //value in [ms].

#define GPS_BSSL_GGA 0x01
#define GPS_BSSL_GLL 0x02
#define GPS_BSSL_GSA 0x04
#define GPS_BSSL_GSV 0x08
#define GPS_BSSL_GNS 0x10
#define GPS_BSSL_RMC 0x20
#define GPS_BSSL_VTG 0x40
#define GPS_BSSL_ZDA 0x80
#define GPS_BSSL_ALL 0xFE

#define GPS_GNS_GPS           0x01
#define GPS_GNS_GLONASS       0x02
#define GPS_GNS_SBAS          0x04
#define GPS_GNS_QZSSL1        0x08

#define GPS_GUSE_DISABLE      0
#define GPS_GUSE_ENABLE       1

typedef enum
{
    GpsResponse_NONE = 0,
    GpsResponse_BUSY,
    GpsResponse_ILLEGAL,
    GpsResponse_TIMEOUT,
    GpsResponse_ERROR,
    GpsResponse_DONE
}GpsResponse;

typedef enum
{
    CmdNum_Version = 0,
    CmdNum_Sleep,
    CmdNum_Wake,
    CmdNum_ColdStart,
    CmdNum_WarmStart,
    CmdNum_HotStart,
    CmdNum_Idle,
    CmdNum_SelectReportTime,
    CmdNum_SelectSatellite,
    CmdNum_SelectSentence,
    CmdNum_BackupData,
    CmdNum_1PPS,
    CmdNum_GUSE,
    CmdNum_None,
    CmdNum_Max
}CmdNum;


typedef enum
{
    GpsState_Init,
    GpsState_Idle,
    GpsState_Exec,
    GpsState_Sleep1,
    GpsState_Sleep2,            // not yet ready
    GpsState_Sleep3,            // not yet ready
    GpsState_Error              // not yet ready
}GpsState;

typedef struct RMCData
{
    uint8_t    UTC[4];                      //HHMMSS(09:58:37.66 ->[09][58][37])
    uint8_t    Date[3];                     //DDMMYY(2017/11/29 ->[29][11][17])
    bool       LatitudeNS;                  //(N->1, S->0)
    uint8_t    LatitudeDegree;              //(2311.37 -> 23 degree -> 23)
    uint32_t   LatitudeMinute;              //(2311.37 -> 11.37 min -> 1137)
    //uint16_t   LatitudeMinute;              //(2311.37 -> 11.37 min -> 1137)
    bool       LongitudaEW;                 //(E->1, W->0)
    uint8_t    LongitudaDegree;             //(12154.21 -> 121 degree -> 121)
    uint32_t   LongitudaMinute;             //(12154.21 -> 54.21 degree -> 5421)
    //uint16_t   LongitudaMinute;             //(12154.21 -> 54.21 degree -> 5421)
    int16_t    Speed;                       //(9.7knot -> 97)
    int16_t    Course;                      //(0.0~360.0 degree -> 0~3600)
    bool       status;                      //(vaild->1, not yet->0)
}RMC_Data;

typedef struct GGAData
{
    uint8_t    UTC[4];
    uint8_t    Date[3];
    bool       LatitudeNS;
    uint8_t    LatitudeDegree;
    //uint16_t   LatitudeMinute;
    uint32_t   LatitudeMinute;
    bool       LongitudaEW;
    uint8_t    LongitudaDegree;
    //uint16_t   LongitudaMinute;
    uint32_t   LongitudaMinute;
    int        Quality;                //(0->not available, 1->fix, 2->fix different GPS, 6->dead reckoning)
    int        NumSatellities;         //(0~32, if only GPS)
    int16_t    HDOP;                   //(2.4 -> 24)
    int16_t    Altitude;               //(hill: 123.6m -> 1236)
    int16_t    GeoidHeight;            //(hole: -16.7m -> -767)
}GGA_Data;

typedef struct GLLData
{
    bool       LatitudeNS;
    uint8_t    LatitudeDegree;
    //uint16_t   LatitudeMinute;
    uint32_t   LatitudeMinute;
    bool       LongitudaEW;
    uint8_t    LongitudaDegree;
    //uint16_t   LongitudaMinute;
    uint32_t   LongitudaMinute;
    uint8_t    UTC[4];
    bool       status;
    char       Indicator;          //('A','D','E','N')
}GLL_Data;

typedef struct GNSData
{
    uint8_t    UTC[4];
    bool       LatitudeNS;
    uint8_t    LatitudeDegree;
    //uint16_t   LatitudeMinute;
    uint32_t   LatitudeMinute;
    bool       LongitudaEW;
    uint8_t    LongitudaDegree;
    //uint16_t   LongitudaMinute;
    uint32_t   LongitudaMinute;
    char       IndicatorGPS;
    char       IndicatorGLONASS;
    int        NumSatellities;
    int16_t    HDOP;
    int16_t    Altitude;
    int16_t    GeoidHeight;
    int16_t    DGPS;             //(3.6 -> 36)
}GNS_Data;

typedef struct GSAData
{
    char       DimensionMode;
    uint8_t    Mode;
    int        UsedSatellite01;
    int        UsedSatellite02;
    int        UsedSatellite03;
    int        UsedSatellite04;
    int        UsedSatellite05;
    int        UsedSatellite06;
    int        UsedSatellite07;
    int        UsedSatellite08;
    int        UsedSatellite09;
    int        UsedSatellite10;
    int        UsedSatellite11;
    int        UsedSatellite12;
    int16_t    PDOP;
    int16_t    HDOP;
    int16_t    VDOP;
}GSA_Data;

typedef struct SVData
{
    int    SatelliteID;
    int    Elevation;
    int    Azimuth;
    int    SNR;
}SV_Data;

typedef struct GSVData
{
    int        TotalNumberSentences;
    int        SentenceNumber;
    int        TotalNumberSatellites;
    SV_Data    SV1;
    SV_Data    SV2;
    SV_Data    SV3;
    SV_Data    SV4;
}GSV_Data;

typedef struct VTGData
{
    int16_t    CourseOverGround_True;
    int16_t    CourseOverGround_Magnetic;
    int16_t    SpeedOverGround_knot;
    int16_t    SpeedOverGround_kmh;
    char       ModeIndicator;
}VTG_Data;

typedef struct ZDAData
{
    uint8_t    UTC[4];
    int        Day;
    int        Month;
    int        Year;
    int        LocalZoneHours;
    int        LocalZoneMinutes;
}ZDA_Data;

typedef struct GPSEngine
{
    GpsState    State;
    CmdNum      Command;
    uint16_t    WordCount;
    RMC_Data    *GPRMC;
    GGA_Data    *GPGGA;
    GLL_Data    *GPGLL;
    GNS_Data    *GPGNS;
}GPS_Engine;

void GpsReset(void);
bool GpsMainInit(GPS_Engine *data, uint8_t bssl, int report_time);
GpsResponse GpsMainLoop(GPS_Engine *data, CmdNum input_cmd, int para1);
void Uart4Rx_Interface(void);
uint16_t GpsRx_Interface(GPS_Engine *cxd_5603, CmdNum *chk_cmd_response, uint8_t *respond);
void GpsCommand_block_until_done(GPS_Engine *, CmdNum, int);
int GpsCommamd_block(GPS_Engine *, CmdNum, int);
void gps_collect(void);

//Group of AnalysisFunction
bool CommandAnalysis(char *data, uint16_t len, CmdNum *command, uint8_t *respond);
#if 0
bool GPRMCAnalysis(char *data, uint16_t len);
bool GPGGAAnalysis(char *data, uint16_t len);
bool GPGLLAnalysis(char *data, uint16_t len);
bool GPGNSAnalysis(char *data, uint16_t len)
#else
bool GPRMCAnalysis(RMC_Data *, char *, uint16_t);
bool GPGGAAnalysis(GGA_Data *, char *, uint16_t);
bool GPGLLAnalysis(GLL_Data *, char *, uint16_t);
bool GPGNSAnalysis(GNS_Data *, char *, uint16_t);
#endif

#endif  // __GPS_DRIVER_H__

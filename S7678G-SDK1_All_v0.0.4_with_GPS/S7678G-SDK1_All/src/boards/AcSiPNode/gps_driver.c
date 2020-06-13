/*

     ___        _____ _ ____
    /   | _____/ ___/(_) __ \
   / /| |/ ___/\__ \/ / /_/ / Tech Co., LTD
  / ___ / /__ ___/ / / ____/
 /_/  |_\___//____/_/_/
    (C)2018 AcSip

Description: GPS receiver driver for SONY CXD5603
*/

#include "board.h"
#include "gps_driver.h"

/*
 * It is just a example. UART4Rx get char and return back by UART4Tx
 */
enum CmdReturn
{
    CmdReturn_Null = 0,
    CmdReturn_Done,
    CmdReturn_Ready,
    CmdReturn_Error
};

extern GPS_Engine S76G_cxd5603;
extern uint32_t LoRaMacState;

//=================================================================
/*
 * Uart4 transparent sending to UART2 (ToDo)
 */
void Uart4Rx_Interface(void)
{
  static uint16_t Read_Ptr = 0, DMA_Ptr = 0;

#if ENABLE_UART4_POLLING
    DMA_Ptr = DMA_BUFFER_SIZE - hdma_rx.Instance->CNDTR;
    if(Read_Ptr > DMA_Ptr) {
        //It mustbe DMA_Ptr cross over DMA_BUFFER_SIZE
        HAL_UART_Transmit_DMA(&Uart4Handle, DMA_UART4_BUFF+Read_Ptr, DMA_BUFFER_SIZE-Read_Ptr);
        Read_Ptr = 0;
    }

    if(DMA_Ptr > Read_Ptr) {
        //DMA data put in buffer, DMA_Ptr++
        HAL_UART_Transmit_DMA(&Uart4Handle, DMA_UART4_BUFF+Read_Ptr, DMA_Ptr-Read_Ptr);
        Read_Ptr = DMA_Ptr;
    }
#endif

    return;
}

//=================================================================
/*
 * Uart4 catch & parsing GPS sentences
 */
#define GPS_STRING_BUFF_SIZE 512
uint16_t GpsRx_Interface(GPS_Engine *cxd_5603, CmdNum *chk_cmd_response, uint8_t *respond)
{
    static char GpsData_Buff[GPS_STRING_BUFF_SIZE];
    static uint16_t Read_Ptr = 0, DMA_Ptr = 0, GPS_Ptr = 0;
    uint16_t i = 0, j;

    DMA_Ptr = DMA_BUFFER_SIZE - hdma_rx.Instance->CNDTR;

    //It must be DMA_Ptr cross over DMA_BUFFER_SIZE
    if(Read_Ptr > DMA_Ptr) {
        if((GPS_Ptr+DMA_BUFFER_SIZE-Read_Ptr) < GPS_STRING_BUFF_SIZE) {
            memcpy(GpsData_Buff+GPS_Ptr, DMA_UART4_BUFF+Read_Ptr, DMA_BUFFER_SIZE-Read_Ptr);
            GPS_Ptr+=DMA_BUFFER_SIZE-Read_Ptr;
        } else {
            memcpy(GpsData_Buff+GPS_Ptr, DMA_UART4_BUFF+Read_Ptr, GPS_STRING_BUFF_SIZE-GPS_Ptr);
            GPS_Ptr = GPS_STRING_BUFF_SIZE;
        }
        Read_Ptr = 0;
    }

    //DMA data put in buffer, DMA_Ptr++
    if(DMA_Ptr > Read_Ptr) {
        if((GPS_Ptr + DMA_Ptr - Read_Ptr) < GPS_STRING_BUFF_SIZE) {
            memcpy(GpsData_Buff+GPS_Ptr, DMA_UART4_BUFF+Read_Ptr, DMA_BUFFER_SIZE-Read_Ptr);
            GPS_Ptr+=DMA_Ptr-Read_Ptr;
        } else {
            memcpy(GpsData_Buff + GPS_Ptr, DMA_UART4_BUFF + Read_Ptr, GPS_STRING_BUFF_SIZE - GPS_Ptr);
            GPS_Ptr = GPS_STRING_BUFF_SIZE;
        }
        Read_Ptr = DMA_Ptr;
    }

    for(i = 0; i < GPS_Ptr; i++) {
        //Got 0x0D staring to parse gps response data
        if(GpsData_Buff[i] == 0x0D && GpsData_Buff[i+1] == 0x0A) {
            if (GpsData_Buff[i+2] == '[') {
                //Handel @VER returning string
                for(j = i+2; j < GPS_Ptr; j++) {
                    if (GpsData_Buff[j] == 0x0D && GpsData_Buff[j+1] == 0x0A) {
                        if(CommandAnalysis(GpsData_Buff+i+2, j-i-2, chk_cmd_response, respond) == true)
                            break;
                    }
                }
            } else {
                if (CommandAnalysis(GpsData_Buff, i, chk_cmd_response, respond) != true) {
                    if (GPRMCAnalysis(cxd_5603->GPRMC, GpsData_Buff, i) != true) {
                        if(GPGGAAnalysis(cxd_5603->GPGGA, GpsData_Buff, i) != true) {
                            if(GPGLLAnalysis(cxd_5603->GPGLL, GpsData_Buff, i) != true) {
                                GPGNSAnalysis(cxd_5603->GPGNS, GpsData_Buff, i);
                            }
                        }
                    }
                }
            }
            GPS_Ptr = 0; 
        }
    }

    if(GPS_Ptr == GPS_STRING_BUFF_SIZE)	
        GPS_Ptr = 0;

    return i;
}

//=================================================================
/*
 * TranslationFunction with all kinds of string
 */
static void Time_Translation(char *data, uint8_t *target)
{
    //Translation:"hhmmss.ss"
    int i;

    for(i=0; i<9; i++) {
        if(i==6) {
            if(data[i] == '.')
                continue;
            else
                i = 10;
        }
        if((data[i]>='0')&&(data[i]<='9'));
        else
            i = 10;
    }

    if(i<10) {
        target[0] = (data[0]-0x30)*10+(data[1]-0x30);
        target[1] = (data[2]-0x30)*10+(data[3]-0x30);
        target[2] = (data[4]-0x30)*10+(data[5]-0x30);
        target[3] = (data[7]-0x30)*10+(data[8]-0x30);
    }

    return;
}

//=================================================================
/*
 * TranslationFunction with all kinds of string
 */
static void Date_Translation(char *data, uint8_t *target)
{
    //Translation:"ddmmyy"
    int i;

    for(i=0; i<6; i++) {
        if((data[i]>='0')&&(data[i]<='9'));
        else
            i = 7;
    }

    if(i<7) {
        target[0] = (data[0]-0x30)*10+(data[1]-0x30);
        target[1] = (data[2]-0x30)*10+(data[3]-0x30);
        target[2] = (data[4]-0x30)*10+(data[5]-0x30);
    }

    return;
}

#define lenglt_max_count 10
#define minute_max_count 5
//static void Coordinate_Translation(char *data, uint8_t *Degree, uint16_t *Minute, bool LAT, bool LON)
static void Coordinate_Translation(char *data, uint8_t *Degree, uint32_t *Minute, bool LAT, bool LON)
{
    //Translation: "12345.78"
    int i=0, minute_count=0, degree=0, minute=0, error=0;

    for(; i<lenglt_max_count; i++) {
        if((data[i]>='0')&(data[i]<='9')) {
            if(minute_count==0) {
                degree = (degree*10)+(data[i]-0x30);
            } else if(minute_count<minute_max_count) {
                minute_count++;
                minute = (minute*10)+(data[i]-0x30);
            }
        } else if(data[i]=='.') {
            if(minute_count==0)
                minute_count = 1;
            else
                error = 1;

            minute = degree%100;
            degree = (degree-minute)/100;
        } else if(data[i]==',') {
            if(minute_count==0) 
                minute_count++;

            for(;minute_count<minute_max_count; minute_count++)	
                minute*=10;

            break;
        } else {
            error = 2;
            break;
        }
    }

    if(LAT) {
        if(degree > 90)
            error = 3;
    }

    if(LON) {
        if(degree > 180)
            error = 3;
    }

    //minute_max_count 4 set 60000. minute_max_count 3 set 6000. 
    if(minute > 600000)
        error = 3;

    if(!error) {
        *Degree = degree;
        *Minute = minute;
    } else if(error==1)  //??
        ;
    else if(error==2)
        ;
    else if(error==3)
        ;

    return;
}

static void Int_Translation(char *data, int *taget)
{
    //Translation: "789"
    int i = 0, answer = 0;
    bool negative = false;

    if(data[0]=='-') {
        i++;
        negative = true;
    }

    for(; i<5; i++) {
        if((data[i]>='0') && (data[i]<='9'))
            answer = (answer*10)+(data[i]-0x30);
        else if(data[i]==',')
            break;
        else
            return;
    }

    if(negative) {
        *taget = -answer; 
    } else {
        *taget = answer; 
    }

    return;
}

static void Float_Translation(char *data, int16_t *taget, int total_len, int float_len)
{
    //Translation: "123.45"
    int i=0, count=0, answer=0, error=0;

    float_len++;
    for(; i<total_len; i++) {
        if((data[i]>='0')&&(data[i]<='9')) {
            if(count!=0)
                count++;
            answer = (answer*10)+(data[i]-0x30);
        } else if(data[i]=='.') {
            if(count==0)
                count++;
            else
                error = 1;
        } else if(data[i]==',') {
            if(count==0) 
                count++;
            for(; count<float_len; count++)	
                answer*=10;

            break;
        } else {
            error = 2;
            break; 
        }
    }

    if(error==0) 
        *taget = answer;

    return;
}

static char CmdTable[CmdNum_Max][7]=
{
    {"[VER]"},{"[SLP]"},{"[WUP]"},{"[GCD]"},{"[GSW]"},{"[GSR]"},{"[GSTP]"},{"[GSOP]"},{"[GNS]"},{"[BSSL]"},{"[BUP]"},{"[GPPS]"},{"[GUSE]"},{"[NONE]"}
};

//=================================================================
/*
 * AnalysisFunction with commands
 */
bool CommandAnalysis(char *data, uint16_t len, CmdNum *cmd, uint8_t *respond)
{
    int i, cmdtype, errtype;

    if(data[0] == '[') {
        for(i = 1; i < len; i++) {
            if(data[i] == ']')
                break;
        }

        if(i < len) {
            i++;
            for(cmdtype = 0; cmdtype < CmdNum_Max; cmdtype++) {
                if(strncmp(data, CmdTable[cmdtype], i) == 0)
                    break;
            }

            if(cmdtype < CmdNum_None) {
                i++;
                *cmd = (CmdNum)cmdtype;
                if(strncmp(data+i, "Done", 4) == 0) {
                    *respond = CmdReturn_Done;
                } else if(strncmp(data+i, "Ready", 5) == 0) {
                    *respond = CmdReturn_Ready;
                } else if(strncmp(data+i, "Err -", 5) == 0) {
                    i+=5;
                    errtype = 0;
                    *respond = CmdReturn_Error;
                    for(; i < len; i++) {
                        errtype = (errtype*10) + (data[i]-0x30);
                    }
                }
            }
        }
        return true;
    }

    return false;
}

//$GPRMC,220516,A,5133.82,N,00042.24,W,173.8,231.8,130694,004.2,W*70
#define RMC_UTC                           1
#define RMC_Status                        2
#define RMC_Latitude                      3
#define RMC_LatitudeNS                    4
#define RMC_Longituda                     5
#define RMC_LongitudaEW                   6
#define RMC_Speed                         7
#define RMC_Course                        8
#define RMC_Date                          9
#define RMC_MagneticVariation             10
#define RMC_MagneticVariationEW           11

//=================================================================
/*
 * AnalysisFunction with all kinds of NMEA
 */
bool GPRMCAnalysis(RMC_Data *RMC, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPRMC,", 7)==0) ||
       (strncmp(data, "$GLRMC,", 7)==0) ||
       (strncmp(data, "$GNRMC,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case RMC_UTC:
                    {
                        Time_Translation(data+i, RMC->UTC);
                        break;
                    }

                    case RMC_Status:
                    {
                        if(data[i]=='A')
                            RMC->status = true;
                        else
                            RMC->status = false;
                        break;
                    }

                    case RMC_Latitude:
                    {
                        Coordinate_Translation(data+i, &RMC->LatitudeDegree, &RMC->LatitudeMinute, true, false);
                        break;
                    }

                    case RMC_LatitudeNS:
                    {
                        if(data[i]=='N')
                            RMC->LatitudeNS = true;
                        else if(data[i]=='S')
                            RMC->LatitudeNS = false;
                        break;
                    }

                    case RMC_Longituda:
                    {
                        Coordinate_Translation(data+i, &RMC->LongitudaDegree, &RMC->LongitudaMinute, false, true);
                        break;
                    }

                    case RMC_LongitudaEW:
                    {
                        if(data[i]=='E')
                            RMC->LongitudaEW = true;
                        else if(data[i]=='W')
                            RMC->LongitudaEW = false;
                        break;
                    }

                    case RMC_Speed:
                    {
                        Float_Translation(data+i, &RMC->Speed, 5, 1);
                        break;
                    }

                    case RMC_Course:
                    {
                        Float_Translation(data+i, &RMC->Course, 5, 1);
                        break;
                    }

                    case RMC_Date:
                    {
                         Date_Translation(data+i, &RMC->Date[0]);
                        break;
                    }

                    case RMC_MagneticVariation:
                    case RMC_MagneticVariationEW:
                    {
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        return true;
    }

    return false;
}

//$GPGGA,010203.04,2399.88,N,12177.66,E,1,10,6.6,426.9,M,426.8,M,x.x,xxxx*hh
#define GGA_UTC                      1
#define GGA_Latitude                 2
#define GGA_LatitudeNS               3
#define GGA_Longituda                4
#define GGA_LongitudaEW              5
#define GGA_Quality                  6
#define GGA_NumSatellities           7
#define GGA_HDOP                     8
#define GGA_Altitude                 9
#define GGA_GeoidHeight              11

bool GPGGAAnalysis(GGA_Data *GGA, char *data, uint16_t len)
{
    int i, count;

    if(strncmp(data, "$GPGGA,", 7)==0) {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case GGA_UTC:
                    {
                        Time_Translation(data+i, GGA->UTC);
                        break;
                    }

                    case GGA_Latitude:
                    {
                        Coordinate_Translation(data+i, &GGA->LatitudeDegree, &GGA->LatitudeMinute, true, false);
                        break;
                    }

                    case GGA_LatitudeNS:
                    {
                        if(data[i]=='N')
                            GGA->LatitudeNS = true;
                        else if(data[i]=='S')
                            GGA->LatitudeNS = false;
                        break;
                    }

                    case GGA_Longituda:
                    {
                        Coordinate_Translation(data+i, &GGA->LongitudaDegree, &GGA->LongitudaMinute, false, true);
                        break;
                    }

                    case GGA_LongitudaEW:
                    {
                        if(data[i]=='E')
                            GGA->LongitudaEW = true;
                        else if(data[i]=='W')
                            GGA->LongitudaEW = false;
                        break;
                    }

                    case GGA_Quality:
                    {
                        Int_Translation(data+i, &GGA->Quality);
                        break;
                    }

                    case GGA_NumSatellities:
                    {
                        Int_Translation(data+i, &GGA->NumSatellities);
                        break;
                    }

                    case GGA_HDOP:
                    {
                        Float_Translation(data+i, &GGA->HDOP, 3, 1);
                        break;
                    }

                    case GGA_Altitude:
                    {
                        Float_Translation(data+i, &GGA->Altitude, 6, 1);
                        break;
                    }

                    case GGA_GeoidHeight:
                    {
                        Float_Translation(data+i, &GGA->GeoidHeight, 6, 1);
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        UartPrint("GPGGA: Lat. & Long. : %c %d %d.%d %c %d %d.%d ,Quality: %d, Number of Sat.: %d, HDOP: %d.%d, Altitude: %d.%d, Geoidal: %d.%d\n",
        (GGA->LatitudeNS?'N':'S'),
        GGA->LatitudeDegree,
        GGA->LatitudeMinute/10000,
        GGA->LatitudeMinute%10000,
        (GGA->LongitudaEW?'E':'W'),
        GGA->LongitudaDegree,
        GGA->LongitudaMinute/10000,
        GGA->LongitudaMinute%10000,
        GGA->Quality,
        GGA->NumSatellities,
        GGA->HDOP/10,
        GGA->HDOP%10,
        GGA->Altitude/10,
        GGA->Altitude%10,
        GGA->GeoidHeight/10,
        GGA->GeoidHeight%10); //Debug print for GGA
        return true;
    }

    return false;
}

//$GPGLL,4916.45,N,12311.12,W,225444,A
#define GLL_Latitude                   1
#define GLL_LatitudeNS                 2
#define GLL_Longituda                  3
#define GLL_LongitudaEW                4
#define GLL_UTC                        5
#define GLL_Status                     6
#define GLL_Indicator                  7

bool GPGLLAnalysis(GLL_Data *GLL, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPGLL,", 7)==0) ||
       (strncmp(data, "$GLGLL,", 7)==0) ||
       (strncmp(data, "$GNGLL,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case GLL_Latitude:
                    {
                        Coordinate_Translation(data+i, &GLL->LatitudeDegree, &GLL->LatitudeMinute, true, false);
                        break;
                    }

                    case GLL_LatitudeNS:
                    {
                        if(data[i]=='N')
                            GLL->LatitudeNS = true;
                        else if(data[i]=='S')
                            GLL->LatitudeNS = false;
                        break;
                    }

                    case GLL_Longituda:
                    {
                        Coordinate_Translation(data+i, &GLL->LongitudaDegree, &GLL->LongitudaMinute, false, true);
                        break;
                    }

                    case GLL_LongitudaEW:
                    {
                        if(data[i]=='E')
                            GLL->LongitudaEW = true;
                        else if(data[i]=='W')
                            GLL->LongitudaEW = false;
                        break;
                    }

                    case GLL_UTC:
                    {
                        Time_Translation(data+i, GLL->UTC);
                        break;
                    }

                    case GLL_Status:
                    {
                        if(data[i]=='A')
                            GLL->status = true;
                        else
                            GLL->status = false;
                        break;
                    }

                    case GLL_Indicator:
                    {
                        GLL->Indicator = data[i];
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        UartPrint("GPGLL: Lat. & Long. : %c %d %d.%d %c %d %d.%d , Status: %c, Mode Indicator: %c\n",
        (GLL->LatitudeNS?'N':'S'),
        GLL->LatitudeDegree,
        GLL->LatitudeMinute/10000,
        GLL->LatitudeMinute%10000,
        (GLL->LongitudaEW?'E':'W'),
        GLL->LongitudaDegree,
        GLL->LongitudaMinute/10000,
        GLL->LongitudaMinute%10000,
        (GLL->status?'A':'V'),
        GLL->Indicator); //Debug print for GLL
        return true;
    }

    return false;
}

//$GPGNS,010203,4916.45,N,12311.12,W,A,10,4.7,426.9,M,426.8,M,0.3,xxxx,E*68
#define GNS_UTC                        1
#define GNS_Latitude                   2
#define GNS_LatitudeNS                 3
#define GNS_Longituda                  4
#define GNS_LongitudaEW                5
#define GNS_Indicator                  6
#define GNS_NumSatellities             7
#define GNS_HDOP                       8
#define GNS_Altitude                   9
#define GNS_GeoidHeight                11
#define GNS_DGPS                       13

bool GPGNSAnalysis(GNS_Data *GNS, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPGNS,", 7)==0) ||
       (strncmp(data, "$GLGNS,", 7)==0) ||
       (strncmp(data, "$GNGNS,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case GNS_UTC:
                    {
                        Time_Translation(data+i, GNS->UTC);
                        break;
                    }

                    case GNS_Latitude:
                    {
                        Coordinate_Translation(data+i, &GNS->LatitudeDegree, &GNS->LatitudeMinute, true, false);
                        break;
                    }

                    case GNS_LatitudeNS:
                    {
                        if(data[i]=='N')
                            GNS->LatitudeNS = true;
                        else if(data[i]=='S')
                            GNS->LatitudeNS = false;
                        break;
                    }

                    case GNS_Longituda:
                    {
                        Coordinate_Translation(data+i, &GNS->LongitudaDegree, &GNS->LongitudaMinute, false, true);
                        break;
                    }

                    case GNS_LongitudaEW:
                    {
                        if(data[i]=='E')
                            GNS->LongitudaEW = true;
                        else if(data[i]=='W')
                            GNS->LongitudaEW = false;
                        break;
                    }

                    case GNS_Indicator:
                    {
                        GNS->IndicatorGPS = data[i];
                        GNS->IndicatorGLONASS = data[i+1];
                        break;
                    }

                    case GNS_NumSatellities:
                    {
                        Int_Translation(data+i, &GNS->NumSatellities);
                        break;
                    }

                    case GNS_HDOP:
                    {
                        Float_Translation(data+i, &GNS->HDOP, 3, 1);
                        break;
                    }

                    case GNS_Altitude:
                    {
                        Float_Translation(data+i, &GNS->Altitude, 6, 1);
                        break;
                    }

                    case GNS_GeoidHeight:
                    {
                        Float_Translation(data+i, &GNS->GeoidHeight, 6, 1);
                        break;
                    }

                    case GNS_DGPS:
                    {
                        Float_Translation(data+i, &GNS->DGPS, 3, 1);
                        break;
                    }

                    default:
                        break;
                }
            }
        }

        UartPrint("GPGNS: Lat. & Long. : %c %d %d.%d %c %d %d.%d , Mode Indicator: %c%c, Number of Sat.: %d, HDOP: %d.%d, Altitude: %d.%d, Geoidal: %d.%d\n",
        (GNS->LatitudeNS?'N':'S'),
        GNS->LatitudeDegree,
        GNS->LatitudeMinute/10000,
        GNS->LatitudeMinute%10000,
        (GNS->LongitudaEW?'E':'W'),
        GNS->LongitudaDegree,
        GNS->LongitudaMinute/10000,
        GNS->LongitudaMinute%10000,
        GNS->IndicatorGPS,
        GNS->IndicatorGLONASS,
        GNS->NumSatellities,
        GNS->HDOP/10,
        GNS->HDOP%10,
        GNS->Altitude/10,
        GNS->Altitude%10,
        GNS->GeoidHeight/10,
        GNS->GeoidHeight%10); //Debug print for GNS
        return true;
    }

    return false;
}

//$GPGSA,A,3,07,08,11,18,27,28,30,,,,,,3.5,2.6,2.4*3C
#define GSA_DimensionMode                 1
#define GSA_Mode                          2
#define GSA_UsedSatellite01               3
#define GSA_UsedSatellite02               4
#define GSA_UsedSatellite03               5
#define GSA_UsedSatellite04               6
#define GSA_UsedSatellite05               7
#define GSA_UsedSatellite06               8
#define GSA_UsedSatellite07               9
#define GSA_UsedSatellite08               10
#define GSA_UsedSatellite09               11
#define GSA_UsedSatellite10               12
#define GSA_UsedSatellite11               13
#define GSA_UsedSatellite12               14
#define GSA_PDOP                          15
#define GSA_HDOP                          16
#define GSA_VDOP                          17

//=================================================================
/*
 * AnalysisFunction with all kinds of NMEA
 */
bool GPGSAAnalysis(GSA_Data *GSA, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPGSA,", 7)==0) ||
       (strncmp(data, "$GLGSA,", 7)==0) ||
       (strncmp(data, "$GNGSA,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case GSA_DimensionMode:
                    {
                        GSA->DimensionMode = data[i];
                        break;
                    }

                    case GSA_Mode:
                    {
                        if(data[i]=='1')
                            GSA->Mode = 1;
                        else if(data[i]=='2')
                            GSA->Mode = 2;
                        else
                            GSA->Mode = 3;
                        break;
                    }

                    case GSA_UsedSatellite01:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite01);
                        break;
                    }

                    case GSA_UsedSatellite02:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite02);
                        break;
                    }

                    case GSA_UsedSatellite03:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite03);
                        break;
                    }

                    case GSA_UsedSatellite04:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite04);
                        break;
                    }

                    case GSA_UsedSatellite05:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite05);
                        break;
                    }

                    case GSA_UsedSatellite06:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite06);
                        break;
                    }

                    case GSA_UsedSatellite07:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite07);
                        break;
                    }

                    case GSA_UsedSatellite08:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite08);
                        break;
                    }

                    case GSA_UsedSatellite09:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite09);
                        break;
                    }

                    case GSA_UsedSatellite10:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite10);
                        break;
                    }

                    case GSA_UsedSatellite11:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite11);
                        break;
                    }

                    case GSA_UsedSatellite12:
                    {
                        Int_Translation(data+i, &GSA->UsedSatellite12);
                        break;
                    }

                    case GSA_PDOP:
                    {
                        Float_Translation(data+i, &GSA->PDOP, 3, 1);
                        break;
                    }

                    case GSA_HDOP:
                    {
                        Float_Translation(data+i, &GSA->HDOP, 3, 1);
                        break;
                    }

                    case GSA_VDOP:
                    {
                        Float_Translation(data+i, &GSA->VDOP, 3, 1);
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        return true;
    }

    return false;
}

//$GPGSV,2,1,08,07,62,267,32,08,42,027,35,11,81,302,37,17,,,26*40
#define GSV_TotalNumberSentences          1
#define GSV_SentenceNumber                2
#define GSV_TotalNumberSatellites         3
#define GSV_SV1_SatelliteID               4
#define GSV_SV1_Elevation                 5
#define GSV_SV1_Azimuth                   6
#define GSV_SV1_SNR                       7
#define GSV_SV2_SatelliteID               8
#define GSV_SV2_Elevation                 9
#define GSV_SV2_Azimuth                   10
#define GSV_SV2_SNR                       11
#define GSV_SV3_SatelliteID               12
#define GSV_SV3_Elevation                 13
#define GSV_SV3_Azimuth                   14
#define GSV_SV3_SNR                       15
#define GSV_SV4_SatelliteID               16
#define GSV_SV4_Elevation                 17
#define GSV_SV4_Azimuth                   18
#define GSV_SV4_SNR                       19

//=================================================================
/*
 * AnalysisFunction with all kinds of NMEA
 */
bool GPGSVAnalysis(GSV_Data *GSV, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPGSV,", 7)==0) ||
       (strncmp(data, "$GLGSV,", 7)==0) ||
       (strncmp(data, "$GNGSV,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case GSV_TotalNumberSentences:
                    {
                        Int_Translation(data+i, &GSV->TotalNumberSentences);
                        break;
                    }

                    case GSV_SentenceNumber:
                    {
                        Int_Translation(data+i, &GSV->SentenceNumber);
                    }

                    case GSV_TotalNumberSatellites:
                    {
                        Int_Translation(data+i, &GSV->TotalNumberSatellites);
                        break;
                    }

                    case GSV_SV1_SatelliteID:
                    {
                        Int_Translation(data+i, &GSV->SV1.SatelliteID);
                        break;
                    }

                    case GSV_SV1_Elevation:
                    {
                        Int_Translation(data+i, &GSV->SV1.Elevation);
                        break;
                    }

                    case GSV_SV1_Azimuth:
                    {
                        Int_Translation(data+i, &GSV->SV1.Azimuth);
                        break;
                    }

                    case GSV_SV1_SNR:
                    {
                        Int_Translation(data+i, &GSV->SV1.SNR);
                        break;
                    }

                    case GSV_SV2_SatelliteID:
                    {
                        Int_Translation(data+i, &GSV->SV2.SatelliteID);
                        break;
                    }

                    case GSV_SV2_Elevation:
                    {
                        Int_Translation(data+i, &GSV->SV2.Elevation);
                        break;
                    }

                    case GSV_SV2_Azimuth:
                    {
                        Int_Translation(data+i, &GSV->SV2.Azimuth);
                        break;
                    }

                    case GSV_SV2_SNR:
                    {
                        Int_Translation(data+i, &GSV->SV2.SNR);
                        break;
                    }

                    case GSV_SV3_SatelliteID:
                    {
                        Int_Translation(data+i, &GSV->SV3.SatelliteID);
                        break;
                    }

                    case GSV_SV3_Elevation:
                    {
                        Int_Translation(data+i, &GSV->SV3.Elevation);
                        break;
                    }

                    case GSV_SV3_Azimuth:
                    {
                        Int_Translation(data+i, &GSV->SV3.Azimuth);
                        break;
                    }

                    case GSV_SV3_SNR:
                    {
                        Int_Translation(data+i, &GSV->SV3.SNR);
                        break;
                    }

                    case GSV_SV4_SatelliteID:
                    {
                        Int_Translation(data+i, &GSV->SV4.SatelliteID);
                        break;
                    }

                    case GSV_SV4_Elevation:
                    {
                        Int_Translation(data+i, &GSV->SV4.Elevation);
                        break;
                    }

                    case GSV_SV4_Azimuth:
                    {
                        Int_Translation(data+i, &GSV->SV4.Azimuth);
                        break;
                    }

                    case GSV_SV4_SNR:
                    {
                        Int_Translation(data+i, &GSV->SV4.SNR);
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        return true;
    }

    return false;
}

//$GPVTG,0.0,T,,M,0.0,N,0.0,K,A*0D
#define VTG_CourseOverGround_True         1
#define VTG_CourseOverGround_Magnetic     3
#define VTG_SpeedOverGround_knot          5
#define VTG_SpeedOverGround_kmh           7
#define VTG_ModeIndicator                 9

//=================================================================
/*
 * AnalysisFunction with all kinds of NMEA
 */
bool GPVTGAnalysis(VTG_Data *VTG, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPVTG,", 7)==0) ||
       (strncmp(data, "$GLVTG,", 7)==0) ||
       (strncmp(data, "$GNVTG,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case VTG_CourseOverGround_True:
                    {
                        Float_Translation(data+i, &VTG->CourseOverGround_True, 5, 1);
                        break;
                    }

                    case VTG_CourseOverGround_Magnetic:
                    {
                        Float_Translation(data+i, &VTG->CourseOverGround_Magnetic, 5, 1);
                        break;
                    }

                    case VTG_SpeedOverGround_knot:
                    {
                        Float_Translation(data+i, &VTG->SpeedOverGround_knot, 5, 1);
                        break;
                    }

                    case VTG_SpeedOverGround_kmh:
                    {
                        Float_Translation(data+i, &VTG->SpeedOverGround_kmh, 5, 1);
                        break;
                    }

                    case VTG_ModeIndicator:
                    {
                        VTG->ModeIndicator = data[i];
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        return true;
    }

    return false;
}

//$GPZDA,075458.00,27,04,2018,,*67
#define ZDA_UTC                           1
#define ZDA_Day                           2
#define ZDA_Month                         3
#define ZDA_Year                          4
#define ZDA_LocalZoneHours                5
#define ZDA_LocalZoneMinutes              6

//=================================================================
/*
 * AnalysisFunction with all kinds of NMEA
 */
bool GPZDAAnalysis(ZDA_Data *ZDA, char *data, uint16_t len)
{
    int i, count;

    if((strncmp(data, "$GPZDA,", 7)==0) ||
       (strncmp(data, "$GLZDA,", 7)==0) ||
       (strncmp(data, "$GNZDA,", 7)==0))
    {
        count = 0;
        for(i=0; i<len; i++) {
            if(data[i]==',') {
                count++;

                //CCW 171228: handle empty message
                if(i<len-1) {
                    if(data[i+1]!=',') {
                        i++;
                    }
                }

                switch(count) {
                    case ZDA_UTC:
                    {
                        Time_Translation(data+i, ZDA->UTC);
                        break;
                    }

                    case ZDA_Day:
                    {
                        Int_Translation(data+i, &ZDA->Day);
                        break;
                    }

                    case ZDA_Month:
                    {
                        Int_Translation(data+i, &ZDA->Month);
                        break;
                    }

                    case ZDA_Year:
                    {
                        Int_Translation(data+i, &ZDA->Year);
                        break;
                    }

                    case ZDA_LocalZoneHours:
                    {
                        Int_Translation(data+i, &ZDA->LocalZoneHours);
                        break;
                    }

                    case ZDA_LocalZoneMinutes:
                    {
                        Int_Translation(data+i, &ZDA->LocalZoneMinutes);
                        break;
                    }

                    default:
                        break;
                }
            }
        }
        return true;
    }

    return false;
}

//=================================================================
//GPS command interface for application
#define GPS_TX_BUFF_SIZE           512
#define GPS_TX_CHECK_COUNT         5000 //@GSW
//#define GPS_TX_CHECK_COUNT         500 //block until status is "DONE" 
//#define GPS_TX_CHECK_COUNT         3000 //@GSR

GpsResponse GpsMainLoop(GPS_Engine *data, CmdNum input_cmd, int para1)
{

    GpsResponse EventType = GpsResponse_NONE;
    unsigned int i;
    static unsigned int running = 0;
    static char TxBuff[GPS_TX_BUFF_SIZE];

    CmdNum ACK_order = CmdNum_None;
    uint8_t ACK_respond = CmdReturn_Null;

    // Prepare GPS command
    if(input_cmd != CmdNum_None) {
        //Check the status of control interface 
        if(data->Command != CmdNum_None)
            EventType = GpsResponse_BUSY;

        //GPS operating state machine
        switch(data->State) {
            case GpsState_Init:
            {
                //Check if the command is "@VER" at Init state
                if (input_cmd != CmdNum_Version)
                    EventType = GpsResponse_ILLEGAL;
                break;
            }

            case GpsState_Idle:
            {
                switch(input_cmd) {
                    case CmdNum_Sleep:    
                    case CmdNum_ColdStart:
                    case CmdNum_WarmStart:
                    case CmdNum_HotStart:
                    case CmdNum_SelectReportTime:
                    case CmdNum_SelectSatellite:
                    case CmdNum_SelectSentence:
                    case CmdNum_BackupData:
                    case CmdNum_Version:    //Add for PV mode testing level shift
                    case CmdNum_Idle:       //Add for issuing Idle command in Idle mode
                    case CmdNum_1PPS:
                    case CmdNum_GUSE:
                        break;
                    default:
                        EventType=GpsResponse_ILLEGAL;
                        break;
                }
                break;
            }

            case GpsState_Exec:
            {
                if (input_cmd != CmdNum_Idle)
                    EventType = GpsResponse_ILLEGAL;
                break;
            }

            case GpsState_Sleep1:
            {
                if (input_cmd != CmdNum_Wake)
                    EventType = GpsResponse_ILLEGAL;
                break;
            }

            default:
                break;
        }
    }

    if(running > GPS_TX_CHECK_COUNT) {
        EventType = GpsResponse_TIMEOUT;
    } else if(running != 0) {
        running++;
    }

    switch(EventType) {
        case GpsResponse_BUSY:
        {
            input_cmd = CmdNum_None;
            break;
        }

        case GpsResponse_ILLEGAL:
        {
            input_cmd = CmdNum_None;
            break;
        }

        case GpsResponse_TIMEOUT:
        {
            input_cmd = CmdNum_None;
            data->Command = CmdNum_None;
            //Fixed bug for reset static variable 'running'
            running = 0;
            break;
        }

        case GpsResponse_ERROR:
        {
            input_cmd = CmdNum_None;
            break;
        }

        case GpsResponse_NONE:
        default:
            break;
    }

    //Prepare command string
    if (input_cmd < CmdNum_None) {
        memset(TxBuff, 0x00, GPS_TX_BUFF_SIZE);
        strcpy(TxBuff, CmdTable[input_cmd]);
        TxBuff[0] = '@';
        data->Command = input_cmd;
        running = 1;
    }

    switch(input_cmd) {
        //Command without parameter
        case CmdNum_Version:
        case CmdNum_Wake:
        case CmdNum_ColdStart:
        case CmdNum_WarmStart:
        case CmdNum_HotStart:
        case CmdNum_BackupData:
        case CmdNum_Idle:
        {
            for(i=0; i<GPS_TX_BUFF_SIZE; i++) {
                if(TxBuff[i]==']') {
                    TxBuff[i] = 0x0d;
                    TxBuff[i+1] = 0x0a;
                    HAL_UART_Transmit_DMA(&Uart4Handle, (uint8_t *)TxBuff, i+2);  //CM, change transfering length
                    break;
                }
            }
            break;
        }

        //One parameter command
        case CmdNum_Sleep:
        case CmdNum_SelectSatellite:
        case CmdNum_SelectSentence:
        case CmdNum_1PPS:
        case CmdNum_GUSE:
        {
            for(i=0; i<GPS_TX_BUFF_SIZE; i++) {
                if(TxBuff[i]==']') {
                    TxBuff[i++] = ' ';
                    char para1_string[3]; 
                    sprintf(para1_string, "%d", para1); 
                    strcpy(TxBuff + i, para1_string);
                    i = i + strlen(para1_string);
                    TxBuff[i++] = 0xd;   
                    TxBuff[i++] = 0x0a;
                    HAL_UART_Transmit_DMA(&Uart4Handle, (uint8_t *)TxBuff, i);
                    break;
                }
            }
            break;
        }

        //Three parameters cpmmand
        case CmdNum_SelectReportTime:
        {
            for(i=0; i<GPS_TX_BUFF_SIZE; i++) {
                if(TxBuff[i]==']') {
                    //default the para1 is always '1'
                    TxBuff[i++] = ' ';
#if GPS_ActiveLowPowerControlMode
                    TxBuff[i++] = '2';    //Changed to 2, for active low power control mode
#else
                    TxBuff[i++] = '1';    //default
#endif
                    //para2, cycle time in ms
                    TxBuff[i++] = ' ';
                    char cycle_time_ms[7];    //max. cycle time = 9,999,999 ms
                    sprintf(cycle_time_ms, "%d", para1);
                    strcpy(TxBuff + i, cycle_time_ms);
                    i = i + strlen(cycle_time_ms);
                    //para3, always specify '0'
                    TxBuff[i++] = ' ';
                    TxBuff[i++] = '0';
                    TxBuff[i++] = 0xd;  
                    TxBuff[i++] = 0x0a;
                    HAL_UART_Transmit_DMA(&Uart4Handle, (uint8_t *)TxBuff, i);
                    break;
                }
            }
            break;
        }
        default:
            break;
    }

    //Determine GPS control state machine
    data->WordCount += GpsRx_Interface(data, &ACK_order, &ACK_respond);

    if(data->Command == ACK_order) {
        if(ACK_respond==CmdReturn_Done) {
            switch(data->Command) {
                case CmdNum_Sleep:
                {
                    data->State = GpsState_Sleep1;
                    break;
                }

                case CmdNum_ColdStart:
                case CmdNum_WarmStart:
                case CmdNum_HotStart:
                {
                    data->State = GpsState_Exec;
                    break;
                } 

                case CmdNum_Version:
                case CmdNum_Wake:
                case CmdNum_Idle:
                {
                    data->State = GpsState_Idle;
                    break;
                }

                default:
                    break;
            }

            EventType = GpsResponse_DONE;
            data->Command = CmdNum_None;
            running = 0;
        }
        else if(ACK_respond==CmdReturn_Ready);   //ToDo?
        else if(ACK_respond==CmdReturn_Error);
    }

    return EventType;
}

//static int GpsCommamd_block(GPS_Engine *data, CmdNum command, int para1)
int GpsCommamd_block(GPS_Engine *data, CmdNum command, int para1)
{
    GpsResponse ans = GpsResponse_NONE;

    while((ans!=GpsResponse_TIMEOUT) && (ans!=GpsResponse_DONE) && (ans!=GpsResponse_ERROR))
        ans = GpsMainLoop(data, command, para1);

    return ans;
}

void GpsCommand_block_until_done(GPS_Engine *data, CmdNum command, int para1)
{
#if 1
    //GpsResponse ans = GpsResponse_NONE;
    GpsResponse ans = GpsResponse_TIMEOUT;

    while(1) { 
        if (ans == GpsResponse_TIMEOUT)
            ans = GpsMainLoop(data, command, para1);
        else if (ans == GpsResponse_DONE)
            break;
        else if (ans == GpsResponse_NONE)
            ans = GpsMainLoop(data, CmdNum_None, 0);
        else if (ans == GpsResponse_ERROR)
            break;
        }
#else
    while(GpsMainLoop(data, command, para1) != GpsResponse_DONE)
        ;
#endif
}

bool GpsMainInit(GPS_Engine *data, uint8_t bssl, int report_time)
{
    //GPS reset
    GpsReset();
    DelayMs(100);

    //Test GPS initial status by @ver command
    GpsCommand_block_until_done(data, CmdNum_Version, 0);
    DelayMs(10);

    //Idle mode
    GpsCommand_block_until_done(data, CmdNum_Idle, 0);
    DelayMs(50);

#if GPS_ActiveLowPowerControlMode == 0
    //GPS NMEA setup - @bssl
    GpsCommand_block_until_done(data, CmdNum_SelectSentence, bssl); 
    DelayMs(100);

    //GPS report cycle setup - @gsop
    GpsCommand_block_until_done(data, CmdNum_SelectReportTime, report_time);
    DelayMs(100);   //Delay 100ms to avoid "Err -16" (BUSY)
#else
    //GPS report cycle setup - @gsop
    GpsCommand_block_until_done(data, CmdNum_SelectReportTime, report_time);
    DelayMs(100);   //Delay 100ms to avoid "Err -16" (BUSY)

    //GPS NMEA setup - @bssl
    GpsCommand_block_until_done(data, CmdNum_SelectSentence, bssl);
    DelayMs(100);

    //Positioning algorithm setting
    GpsCommand_block_until_done(data, CmdNum_GUSE, GPS_GUSE_ENABLE);
    DelayMs(100);
#endif

    //GPS Satellite setup - @gns
    GpsCommand_block_until_done(data, CmdNum_SelectSatellite, GPS_GNS_GPS + GPS_GNS_GLONASS);      //Hybrid = GPS + GLONAS
    //GpsCommand_block_until_done(data, CmdNum_SelectSatellite, GPS_GNS_GPS);      //GPS
    //GpsCommand_block_until_done(data, CmdNum_SelectSatellite, GPS_GNS_GLONASS);  //GLONAS
    DelayMs(100);

    //GPS start
    GpsCommand_block_until_done(data, CmdNum_HotStart, 0);   //Hot start is the default operating mode
    //GpsCommand_block_until_done(data, CmdNum_WarmStart, 0);

    return true;
}

//SONY CXD 5603 RST_X, GPS nRST control
void GpsReset(void)
{
    Gpio_t GPS_nRST;

    //Set GPS Reset Pin as 0
    GpioInit( &GPS_nRST, GPS_RST_X, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    //Scope shows 1.12s (Low Period)
    DelayMs( 200 );

    //Set GPS Reset Pin as 1
    GpioWrite( &GPS_nRST, 1 );
}

void gps_collect(void)
{
    static uint16_t previous_Wordcount = 0;
    static GpsResponse gps_status = GpsResponse_NONE;

    // Just return if not joined or TX-ing
    if (LoRaMacState != 0x00000000) {
        return;
    }

    //Checking GPS response
    switch(gps_status) {
        case GpsResponse_NONE:
        case GpsResponse_TIMEOUT:
            gps_status = GpsMainLoop(&S76G_cxd5603, CmdNum_None, 0);

            if (S76G_cxd5603.WordCount != previous_Wordcount && S76G_cxd5603.GPRMC->status) {
                //GPS coordinate is avaliable
                previous_Wordcount = S76G_cxd5603.WordCount;
            }
            break;

        case GpsResponse_DONE:
             break;

        case GpsResponse_BUSY:
             break;

        case GpsResponse_ILLEGAL:
        case GpsResponse_ERROR:
            break;
    }
}

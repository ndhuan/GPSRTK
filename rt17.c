
/*
| Included files:
*/

#include "rtk.h"


/*
| Constant definitions:
*/

#define STX           2    /* Start of packet character */
#define ETX           3    /* End of packet character */
#define GENOUT        0x40 /* General Serial Output Format (GSOF) */
#define RETSVDATA     0x55 /* Satellite information reports */
#define RAWDATA       0x57 /* Position or real-time survey data report */
#define BIG_ENDIAN    1    /* Big-endian platform or data stream */
#define LITTLE_ENDIAN 2    /* Little-endian platform or data stream */

/*
| Record Interpretation Flags bit masks:
*/
#define M_CONCISE     1    /* Concise format */
#define M_ENHANCED    2    /* Enhanced record with real-time flags and IODE information */

/*
| Raw->flag bit definitions:
*/
#define M_WEEK_OPTION 1    /* Raw=>week contains GPS week number set by WEEK=n option */
#define M_WEEK_SCAN   2    /* WEEK=n option already looked for, no need to do it again */

/*
| Data conversion macros:
*/

#define I1(p) (*((char*)(p)))          /* One byte signed integer */
#define U1(p) (*((unsigned char*)(p))) /* One byte unsigned integer */
#define I2(p,e) read_i2(p,e)           /* Two byte signed integer */
#define U2(p,e) read_u2(p,e)           /* Two byte unsigned integer */
#define I4(p,e) read_i4(p,e)           /* Four byte signed integer */
#define U4(p,e) read_u4(p,e)           /* Four byte unsigned integer */
#define R4(p,e) read_r4(p,e)           /* IEEE S_FLOAT floating point number */
#define R8(p,e) read_r8(p,e)           /* IEEE T_FLOAT floating point number */


/*
| Internal structure definitions.
*/
typedef union {unsigned short u2; unsigned char c[2];} ENDIAN_TEST; 


/*
| Static global literals:
*/


/*
| Static global variables:
*/


/*
| Internal private function forward declarations (in alphabetical order):
*/

static int check_packet_checksum(raw_t *raw);
static void clear_message_buffer(raw_t *raw);
static void clear_packet_buffer(raw_t *raw);
static void decode_genout(raw_t *raw, int endian);
static Error decode_gps_ephemeris(raw_t *raw, int endian);
static void decode_gsof_1(raw_t *raw, unsigned char *p, int endian);
static void decode_gsof_16(raw_t *raw, unsigned char *p, int endian);
static void decode_gsof_26(raw_t *raw, unsigned char *p, int endian);
static void decode_gsof_41(raw_t *raw, unsigned char *p, int endian);
static Error decode_ion_utc_data(raw_t *raw, int endian);
static Error decode_rawdata(raw_t *raw, int endian);
static Error decode_retsvdata(raw_t *raw, int endian);
static Error decode_type_17(raw_t *raw, unsigned int rif, int endian);
static Error decode_type_29(raw_t *raw, int endian);
static int get_week(raw_t *raw, double receive_time);
static short read_i2(unsigned char *p, int endian);
static int read_i4(unsigned char *p, int endian);
static float read_r4(unsigned char *p, int endian);
static double read_r8(unsigned char *p, int endian);
static unsigned short read_u2(unsigned char *p, int endian);
static unsigned int read_u4(unsigned char *p, int endian);
static void set_week(raw_t *raw, int week, double receive_time);
static int sync_packet(raw_t *raw, unsigned char data);
static void unwrap_rawdata(raw_t *raw, unsigned int *rif);
static void unwrap_genout(raw_t *raw);

extern Error input_rt17(raw_t *raw, unsigned char data)
{
    Error status = 0;
    unsigned int page, pages, reply;
                              
    /*
    | If no current packet.
    */
    if (raw->pbyte == 0)
    {   
        /*
        | Find something that looks like a packet.
        */
        if (sync_packet(raw, data))
        {
            /*
            | Found one.
            */
            raw->plen = 4 + raw->pbuff[3] + 2; /* 4 (header) + length + 2 (trailer) */
            raw->pbyte = 4; /* We now have four bytes in the packet buffer */
        }
                
        /*
        | Continue reading the rest of the packet from the stream.
        */
        return INCOMPLETE;
    }

    /*
    | Store the next byte of the packet.
    */
    raw->pbuff[raw->pbyte++] = data;

    /*
    | Keep storing bytes into the current packet
    | until we have what we think are all of them.
    */
    if (raw->pbyte < raw->plen)
        return INCOMPLETE;

    /*
    | At this point we think have an entire packet.
    | The prospective packet must end with an ETX.
    */
    if (raw->pbuff[raw->plen-1] != ETX)
    {
        clear_packet_buffer(raw);
        return INCOMPLETE;
    }

    /*
    | We do indeed have an entire packet.
    | Check the packet checksum.
    */
    if (!check_packet_checksum(raw))
    {

        clear_packet_buffer(raw);
        return INCOMPLETE;
    }

    /*
    | If this is a SVDATA packet, then process it immediately.
    */
    if (raw->pbuff[2] == RETSVDATA)
    {
        status = decode_retsvdata( raw, BIG_ENDIAN );//motorola format
        clear_packet_buffer(raw);
        return (status);
    }
        
    /*
    | Accumulate a sequence of RAWDATA packets (pages).
    */
    if (raw->pbuff[2] == RAWDATA)
    {
        page = raw->pbuff[5] >> 4;
        pages = raw->pbuff[5] & 15;
        reply = raw->pbuff[6];

        /*
        | If this is the first RAWDATA packet in a sequence of RAWDATA packets,
        | then make sure it's page one and not a packet somewhere in the middle.
        | If not page one, then skip it and continue reading from the stream
        | until we find one that starts at page one. Otherwise make sure it is
        | a part of the same requence of packets as the last one, that it's
        | page number is in sequence.
        */
        if (raw->nbyte == 0)
        {       
            if (page != 1)
            {
                clear_packet_buffer(raw);
                return INCOMPLETE;
            }

            raw->reply = raw->pbuff[6];
        }
        else if ((reply != raw->reply) || (page != (raw->page + 1)))
        {
            clear_message_buffer(raw);
            clear_packet_buffer(raw);
            return INCOMPLETE;
        }
        
        /*
        | Check for raw->buff buffer overflow.
        */
        if ((raw->nbyte + raw->pbyte) > MAX_RAW_LEN)
        {
            clear_message_buffer(raw);
            clear_packet_buffer(raw);
            return INCOMPLETE; 
        }

        memcpy(raw->buff + raw->nbyte, raw->pbuff, raw->pbyte);
        raw->nbyte += raw->pbyte;
        raw->len += raw->plen;
        clear_packet_buffer(raw);

        if (page == pages)
        {
            status = decode_rawdata( raw, BIG_ENDIAN );
            clear_message_buffer(raw);
            return (status);
        }

        raw->page = page;

        return INCOMPLETE;
    }

     /*
    | Accumulate a sequence of GENOUT (GSOF) packets (pages).
    */
    if (raw->pbuff[2] == GENOUT)
    {
        reply = raw->pbuff[4];
        page = raw->pbuff[5];
        pages = raw->pbuff[6];

        /*
        | If this is the first GENOUT packet in a sequence of GENOUT packets,
        | then make sure it's page zero and not a packet somewhere in the middle.
        | If not page zero, then skip it and continue reading from the stream
        | until we find one that starts at page zero. Otherwise make sure it is
        | a part of the same requence of packets as the last one, that it's
        | page number is in sequence.
        */
        if (raw->nbyte == 0)
        {       
            if (page != 0)
            {
                clear_packet_buffer(raw);
                return INCOMPLETE;
            }

            raw->reply = raw->pbuff[4];
        }
        else if ((reply != raw->reply) || (page != (raw->page + 1)))
        {
            clear_message_buffer(raw);
            clear_packet_buffer(raw);
            return INCOMPLETE;
        }
        
        /*
        | Check for raw->buff buffer overflow.
        */
        if ((raw->nbyte + raw->pbyte) > MAX_RAW_LEN)
        {
            clear_message_buffer(raw);
            clear_packet_buffer(raw);
            return INCOMPLETE; 
        }

        memcpy(raw->buff + raw->nbyte, raw->pbuff, raw->pbyte);
        raw->nbyte += raw->pbyte;
        raw->len += raw->plen;
        clear_packet_buffer(raw);

        if (page == pages)
        {
            decode_genout( raw, BIG_ENDIAN );
            clear_message_buffer(raw);
            return INCOMPLETE;
        }

        raw->page = page;

        return INCOMPLETE;
    }

    /*
    | If we fall through to here, then the packet is not one that we support
    | (and hence we can't really even get here). Dump the packet on the floor
    | and continue reading from the stream.
    */
    clear_packet_buffer(raw);
    return INCOMPLETE;
}

static int check_packet_checksum(raw_t *raw)
{
    unsigned char checksum = 0;
    unsigned char *p = &raw->pbuff[1];       /* Starting with status */
    unsigned int length = raw->pbuff[3] + 3; /* status, type, length, data */
  
    /*
    | Compute the packet checksum.
    */
    while (length > 0)
    {
        checksum += *p++;
        length--;
    }

    /*
    | Make sure our computed checksum matches the one at the end of the packet.
    | (Note that the above loop by design very conveniently left *p pointing
    |  to the checksum byte at the end of the packet.)
    */ 
    return (checksum == *p);
}
static void clear_message_buffer(raw_t *raw)
{
    int i;

    for (i = 0; i < 4; i++)
        raw->buff[i] = 0;

    raw->len = raw->nbyte = 0;
    raw->reply = 0;
}
/*
| Function: decode_retsvdata
| Purpose:  Decode an SVDATA packet 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   Raw->pbuff[]
|   Raw->plen
|
| Implicit outputs:
|
|   Raw->pbuff[]
|   Raw->plen
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
*/
static void clear_packet_buffer(raw_t *raw)
{
    int i;

    for (i = 0; i < 4; i++)
        raw->pbuff[i] = 0;

    raw->plen = raw->pbyte = 0;
}
static void decode_genout(raw_t *raw, int endian)
{
    static const char *rt[] = { NULL,                                   /* 00 */
                                "Position Time",                        /* 01 */
                                "Latitude Longitude Height",            /* 02 */
                                "ECEF Position",                        /* 03 */
                                "Local Datum LLH Position",             /* 04 */
                                "Local Zone ENU Position",              /* 05 */
                                "ECEF Delta",                           /* 06 */
                                "Tangent Plane Delta",                  /* 07 */
                                "Velocity Data",                        /* 08 */
                                "PDOP Information",                     /* 09 */
                                "Clock Information",                    /* 10 */
                                "Position VCV Information",             /* 11 */
                                "Position Sigma Information",           /* 12 */
                                "SV Brief Information",                 /* 13 */
                                "SV Detailed Information",              /* 14 */
                                "Receiver Serial Number",               /* 15 */
                                "Current Time",                         /* 16 */
                                NULL,                                   /* 17 */
                                NULL,                                   /* 18 */
                                NULL,                                   /* 19 */
                                NULL,                                   /* 20 */
                                NULL,                                   /* 21 */
                                NULL,                                   /* 22 */
                                NULL,                                   /* 23 */
                                NULL,                                   /* 24 */
                                NULL,                                   /* 25 */
                                "Position Time UTC",                    /* 26 */
                                "Attitude Information",                 /* 27 */
                                NULL,                                   /* 28 */
                                NULL,                                   /* 29 */
                                NULL,                                   /* 30 */
                                NULL,                                   /* 31 */
                                NULL,                                   /* 32 */
                                "All SV Brief Information",             /* 33 */
                                "All SV Detailed Information",          /* 34 */
                                "Received Base Information",            /* 35 */
                                NULL,                                   /* 36 */
                                "Battery and Memory Information",       /* 37 */
                                NULL,                                   /* 38 */
                                NULL,                                   /* 39 */
                                "L-Band Status Information",            /* 40 */
                                "Base Position and Quality Indicator" };/* 41 */

    int input_len;
    unsigned char len, recordtype, *p;
    char *recordtype_s = NULL;
 
   /*
    | Reassemble origional message by removing packet headers,
    | trailers and page framing.
    */
    unwrap_genout(raw);

    p = raw->buff;
    input_len = raw->len;

    while (input_len)
    {
        recordtype = p[0];
        len = p[1];

        if (recordtype < (sizeof(rt) / sizeof(char*)))
            recordtype_s = (char*) rt[recordtype];

        if (!recordtype_s)
            recordtype_s = "Unknown";
      
        /*
        | Process (or possibly ignore) the message.
        */
        switch (recordtype)
        {
        case 1:
            decode_gsof_1(raw, p, endian);
            break;
        case 16:
            decode_gsof_16(raw, p, endian);
            break;
        case 26:
            decode_gsof_26(raw, p, endian);
            break;
        case 41:
            decode_gsof_41(raw, p, endian);
            break;
        default:
					{
						//trace(4, "RT17: GSOF message not processed.\n");    
					}
        }

        len += 2;
        p += len;
        input_len -= len;
    }
}


/*
| Function: decode_gps_ephemeris
| Purpose:  Decode a GPS Ephemeris record
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw = Receiver raw data control structure [Input]
|   Endian = Endianness indicator             [Input]
|
| Implicit Inputs:
|
|   Raw->pbuff[]
|   Raw->plen
|
| Implicit outputs:
|
|   Raw->pbuff[]
|   Raw->plen
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   See ICD-GPS-200C.PDF for documentation of the GPS satellite ephemeris.
|   See reference #1 above for documentation of the RETSVDATA GPS Ephemeris.
*/

static Error decode_gps_ephemeris(raw_t *raw, int e)
{
    unsigned char *p = raw->pbuff;
    int prn, sat, toc, tow;
    unsigned int flags, toe;
    double sqrtA;
    eph_t eph={0};

    if (raw->plen < 182)
    {
        return (LEN_ERROR);
    }

    prn = U1(p+5);

    if (!(sat=satno(SYS_GPS, prn)))
    {
        return EPH_ERROR;
    }
 
    eph.week  = U2(p+6,e);   /* 006-007: Ephemeris Week number (weeks) */
    eph.iodc  = U2(p+8,e);   /* 008-009: IODC */ 
    /* Reserved byte */      /* 010-010: RESERVED */
    eph.iode  = U1(p+11);    /* 011-011: IODE */
    tow       = I4(p+12,e);  /* 012-015: TOW */
    toc       = I4(p+16,e);  /* 016-019: TOC (seconds) */
    toe       = U4(p+20,e);  /* 020-023: TOE (seconds) */                                   
    eph.tgd[0]= R8(p+24,e);  /* 024-031: TGD (seconds) */
    eph.f2    = R8(p+32,e);  /* 032-029: AF2 (seconds/seconds^2) */
    eph.f1    = R8(p+40,e);  /* 040-047: AF1 (seconds/seconds) */
    eph.f0    = R8(p+48,e);  /* 048-055: AF0 (seconds) */
    eph.crs   = R8(p+56,e);  /* 056-063: CRS (meters) */
    eph.deln  = R8(p+64,e);  /* 064-071: DELTA N (semi-circles/second) */
    eph.M0    = R8(p+72,e);  /* 072-079: M SUB 0 (semi-circles) */
    eph.cuc   = R8(p+80,e);  /* 080-087: CUC (semi-circles) */
    eph.e     = R8(p+88,e);  /* 088-095: ECCENTRICITY (dimensionless) */
    eph.cus   = R8(p+96,e);  /* 096-103: CUS (semi-circles) */
    sqrtA     = R8(p+104,e); /* 104-111: SQRT A (meters ^ 0.5) */
    eph.cic   = R8(p+112,e); /* 112-119: CIC (semi-circles) */
    eph.OMG0  = R8(p+120,e); /* 120-127: OMEGA SUB 0 (semi-circles) */
    eph.cis   = R8(p+128,e); /* 128-135: CIS (semi-circlces) */
    eph.i0    = R8(p+136,e); /* 136-143: I SUB 0 (semi-circles) */
    eph.crc   = R8(p+144,e); /* 144-151: CRC (meters) */
    eph.omg   = R8(p+152,e); /* 152-159: OMEGA (semi-circles?) */
    eph.OMgd  = R8(p+160,e); /* 160-167: OMEGA DOT (semi-circles/second) */
    eph.idot  = R8(p+168,e); /* 168-175: I DOT (semi-circles/second) */
    flags     = U4(p+176,e); /* 176-179: FLAGS */
  
    /*
    | Multiply these by PI to make ICD specified semi-circle units into radian
    | units for RTKLIB.
    */
    eph.deln *= SC2RAD;
    eph.i0   *= SC2RAD;
    eph.idot *= SC2RAD;
    eph.M0   *= SC2RAD;
    eph.omg  *= SC2RAD;
    eph.OMG0 *= SC2RAD;
    eph.OMgd *= SC2RAD;

    /*
    | As specifically directed to do so by Reference #1, multiply these by PI
    | to make the non-standard Trimble specified semi-circle units into radian
    | units, which is what ICD-GPS-200C calls for and also what RTKLIB needs.
    */
    eph.cic *= SC2RAD;
    eph.cis *= SC2RAD;
    eph.cuc *= SC2RAD;
    eph.cus *= SC2RAD;
 
    /*
    | Select the correct curve fit interval as per ICD-GPS-200 sections
    | 20.3.3.4.3.1 and 20.3.4.4 using IODC, fit flag and Table 20-XII.
    */
    if (flags & 1024)  /* Subframe 2, word 10, bit 17 (fit flag) */
    {
        if ((eph.iodc >= 240) && (eph.iodc <= 247))
            eph.fit = 8;
        else if (((eph.iodc >= 248) && (eph.iodc <= 255)) || (eph.iodc == 496))
            eph.fit = 14;
        else if ((eph.iodc >= 497) && (eph.iodc <= 503))
            eph.fit = 26;
        else if ((eph.iodc >= 504) && (eph.iodc <= 510))
            eph.fit = 50;
        else if ((eph.iodc == 511) || ((eph.iodc >= 752) && (eph.iodc <= 756)))
            eph.fit = 74;
        else if ((eph.iodc >= 757) && (eph.iodc <= 763))
            eph.fit = 98;
        else if (((eph.iodc >= 764) && (eph.iodc <= 767)) || ((eph.iodc >= 1008) && (eph.iodc <= 1010)))
            eph.fit = 122;
        else if ((eph.iodc >= 1011) && (eph.iodc <= 1020))
            eph.fit = 146;
        else
            eph.fit = 6;
    }
    else
        eph.fit = 4;

    eph.svh   = (flags >> 4) & 127; /* Subframe 1, word 3, bits 17?22, SV health from ephemeris */
    eph.sva   = (flags >> 11) & 15; /* Subframe 1, word 3, bits 13?16, User Range Accuracy index */     

    eph.A     = sqrtA * sqrtA;

    eph.toes  = toe;
    eph.toc   = gpst2time(eph.week, toc);
    eph.toe   = gpst2time(eph.week, toe);
    eph.ttr   = gpst2time(eph.week, tow);

    if (1)
    {
        if (eph.iode == raw->nav.eph[sat-1].iode)
            return (UNCHANGE); /* unchanged */
    }

    eph.sat = sat;
    raw->nav.eph[sat-1] = eph;
    raw->ephsat = sat;

    return (EPHEMERIS);
}



/*
| Function: decode_gsof_1
| Purpose:  Decode a Position Time GSOF message 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   p      = Buffer pointer                      [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit outputs:
|
|   Raw->week
|
| Return Value:
|
|   <none>
|
| Design Issues:
|
|   We don't actually "decode" this message per-se.
|   We're only interested in the GPS week number.
|
*/
static void decode_gsof_1(raw_t *raw, unsigned char *p, int endian)
{

    if (p[1] < 7)
    {

		}
    else
        set_week(raw, I2(p+6, endian), (double) I4(p+2, endian));
}
 
/*
| Function: decode_gsof_16
| Purpose:  Decode a Current Time GSOF message 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   p      = Buffer pointer                      [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit outputs:
|
|   Raw->week
|
| Return Value:
|
|   <none>
|
| Design Issues:
|
|   We don't actually "decode" this message per-se.
|   We're only interested in the GPS week number.
|
*/
static void decode_gsof_16(raw_t *raw, unsigned char *p, int endian)
{
    if (p[1] < 10)
    {    
		}
    else if (U1(p+10) & 1) /* If week and milliseconds of week are valid */
        set_week(raw, I2(p+6, endian), (double) I4(p+2, endian));
}

/*
| Function: decode_gsof_26
| Purpose:  Decode a Position Time UTC GSOF message 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   p      = Buffer pointer                      [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit outputs:
|
|   Raw->week
|
| Return Value:
|
|   <none>
|
| Design Issues:
|
|   We don't actually "decode" this message per-se.
|   We're only interested in the GPS week number.
|
*/
static void decode_gsof_26(raw_t *raw, unsigned char *p, int endian)
{
    if (p[1] < 7)
    {
			//trace( 2, "RT17: GSOF Position Time UTC message "
      //           "record length %d < 7 bytes. Record discarded.\n", p[1] );
		}
    else
        set_week(raw, I2(p+6, endian), (double) I4(p+2, endian));
}

/*
| Function: decode_gsof_41
| Purpose:  Decode a Base Position and Quality Indicator GSOF message 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   p      = Buffer pointer                      [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit outputs:
|
|   Raw->week
|
| Return Value:
|
|   <none>
|
| Design Issues:
|
|   We don't actually "decode" this message per-se.
|   We're only interested in the GPS week number.
|
*/
static void decode_gsof_41(raw_t *raw, unsigned char *p, int endian)
{
    if (p[1] < 7)
    {    //trace( 2, "RT17: GSOF Base Position and Quality Indicator message "
         //          "record length %d < 7 bytes. Record discarded.\n", p[1] );
		}
    else 
        set_week(raw, I2(p+6, endian), (double) I4(p+2, endian));
}



/*
| Function: decode_ion_utc_data
| Purpose:  Decode an ION / UTC data record
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw  = Receiver raw data control structure [Input]
|   Endian = Endianness indicator              [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit outputs:
|
|   Raw->buff[]
|   Raw->len
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   See ICD-GPS-200C.PDF for documetation of GPS ION / UTC data.
|   See reference #1 above for documentation of RETSVDATA and ION / UTC data.
*/
static Error decode_ion_utc_data(raw_t *raw, int e)
{
    int week;
    unsigned char *p = raw->pbuff;
   
    //trace(4, "RT17: decode_ion_utc_data, length=%d.\n", raw->plen);

    if (raw->plen < 129)
    {
        //trace( 2, "RT17: RETSVDATA packet length %d < 129 bytes. "
        //          "GPS ION / UTC data packet discarded.\n", raw->plen );
        return (LEN_ERROR);
    }

    /*
    | ION / UTC data does not have the current GPS week number. Punt!
    */
		if (raw->time.time)
			return TIMING_ERROR;
    week = get_week(raw, 0.0);
 
    raw->nav.ion_gps[0] = R8(p+6,e);   /* 006?013: ALPHA 0 (seconds) */
    raw->nav.ion_gps[1] = R8(p+14,e);  /* 014?021: ALPHA 1 (seconds/semi-circle) */
    raw->nav.ion_gps[2] = R8(p+22,e);  /* 022?029: ALPHA 2 (seconds/semi-circle)^2 */ 
    raw->nav.ion_gps[3] = R8(p+30,e);  /* 030?037: ALPHA 3 (seconds/semi-circle)^3 */
    raw->nav.ion_gps[4] = R8(p+38,e);  /* 038?045: BETA 0  (seconds) */
    raw->nav.ion_gps[5] = R8(p+46,e);  /* 046?053: BETA 1  (seconds/semi-circle) */
    raw->nav.ion_gps[6] = R8(p+54,e);  /* 054?061: BETA 2  (seconds/semi-circle)^2 */
    raw->nav.ion_gps[7] = R8(p+62,e);  /* 062?069: BETA 3  (seconds/semi-circle)^3 */
    raw->nav.utc_gps[0] = R8(p+70,e);  /* 070?077: ASUB0   (seconds)*/ 
    raw->nav.utc_gps[1] = R8(p+78,e);  /* 078?085: ASUB1   (seconds/seconds) */     
    raw->nav.utc_gps[2] = R8(p+86,e);  /* 086?093: TSUB0T */ 
    raw->nav.utc_gps[3] = week;
    raw->nav.leaps =(int) R8(p+94,e);  /* 094?101: DELTATLS (seconds) */
    /* Unused by RTKLIB R8 */          /* 102?109: DELTATLSF */
    /* Unused by RTKLIB R8 */          /* 110?117: IONTIME */
    /* Unused by RTKLIB U1 */          /* 118-118: WNSUBT */
    /* Unused by RTKLIB U1 */          /* 119-119: WNSUBLSF */
    /* Unused by RTKLIB U1 */          /* 120-120: DN */
    /* Reserved six bytes */           /* 121?126: RESERVED */
   
   return (NO_ERROR1);
}

/*
| Function: decode_rawdata
| Purpose:  Decode an RAWDATA packet sequence 
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw = Receiver raw data control structure [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit outputs:
|
|   Raw->buff[]
|   Raw->len
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
*/
static Error decode_rawdata(raw_t *raw, int endian)
{
    static const char *rt[] = { "Real-time GPS Survey Data (type 17)",  /* 00 */
                                "Position Record (type 11)",            /* 01 */
                                "Event Mark",                           /* 02 */
                                NULL,                                   /* 03 */
                                NULL,                                   /* 04 */
                                NULL,                                   /* 05 */
                                "Real-time GNSS Survey Data (type 27)", /* 06 */
                                "Enhanced Position Record (type 29)" }; /* 07 */

    Error status = 0;
    unsigned int rif;
    char *recordtype_s = NULL;
    unsigned char recordtype = raw->buff[4];
 
    if (recordtype < (sizeof(rt) / sizeof(char*)))
        recordtype_s = (char*) rt[recordtype];

    if (!recordtype_s)
        recordtype_s = "Unknown";
      
    /*
    | Reassemble origional message by removing packet headers,
    | trailers and page framing.
    */
    unwrap_rawdata(raw, &rif);

    /*
    | Process (or possibly ignore) the message.
    */
    switch (recordtype)
    {
    case 0:
        status = decode_type_17(raw, rif, endian);
        break;
    case 7:
        status = decode_type_29(raw, endian);
        break;
    default:
        break;
    }

    return (status);
}


static Error decode_retsvdata(raw_t *raw, int endian)
{
    static const char *st[] = { "SV Flags",                         /* 00 */
                                "GPS Ephemeris",                    /* 01 */
                                "GPS Almanac","ION / UTC Data",     /* 02 */
                                "Disable Satellite (Depreciated)",  /* 03 */
                                "Enable Satellite (Depreciated)",   /* 04 */
                                NULL,                               /* 05 */
                                "Extended GPS Almanac",             /* 06 */
                                "GLONASS Almanac",                  /* 07 */
                                "GLONASS Ephemeris",                /* 08 */
                                NULL,                               /* 09 */
                                "Galileo Ephemeris",                /* 10 */
                                "Galileo Almanac",                  /* 11 */
                                NULL,                               /* 12 */
                                "QZSS Ephemeris",                   /* 13 */
                                NULL,                               /* 14 */
                                "QZSS Almanac",                     /* 15 */
                                NULL,                               /* 16 */
                                NULL,                               /* 17 */
                                NULL,                               /* 18 */
                                "SV Flags",                         /* 19 */
                                "BeiDou Ephemeris",                 /* 20 */
                                "BeiDou Almanac" };                 /* 21 */
 
    Error status=0;
    char *subtype_s = NULL;
    unsigned char subtype = raw->pbuff[4];

    if (subtype < (sizeof(st) / sizeof(char*)))
        subtype_s = (char*) st[subtype];
  
    if (!subtype_s)
        subtype_s = "Unknown";
      
    /*
    | Process (or possibly ignore) the message.
    */
    switch (subtype)
    {
    case 1:
        status = decode_gps_ephemeris(raw, endian);
        break;
    case 3:
        status = decode_ion_utc_data(raw, endian);
        break;
    default:
        break;
    }
    return (status);
}



/*
| Function: decode_type_17
| Purpose:  Decode Real-Time survey data (record type 17)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   RIF    = Rcord Interpretation Flags          [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit outputs:
|
|   Raw->buff[]
|   Raw->len
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   Handles expanded and concise formats with and without enhanced record data.
*/
static Error decode_type_17(raw_t *raw, unsigned int rif, int e)
{
    unsigned char *p = raw->buff;
    double clock_offset, receive_time;
    double l1_carrier, l1_doppler, l1_pseudorange;
    //double l2_carrier, l2_doppler, l2_pseudorange;
    unsigned char l1_code, l1_lli, l1_snr;
    //unsigned char l2_code, l2_lli, l2_snr;
    int flags1, flags2, flag_status, i, j, nsat, prn, sat, week, n = 0;
    gtime_t time;

    receive_time = R8(p,e); p += 8; /* Receive time within the current GPS week (ms). */
    clock_offset = R8(p,e); p += 8; /* Clock offset value (ms). 0.0 = not known */ 

    //if (strstr(raw->opt, "-CO"))
        receive_time += clock_offset;

    /*
    | The observation data does not have the current GPS week number. Punt!
    */
    if (raw->time.time == 0)
			return TIMING_ERROR;
		week = get_week(raw, receive_time);
		
    /*
    | Turn the receive time in milliseconds since the start of the current GPS week
    | into time in seconds and fractional seconds.
    */   
    time = gpst2time(week, receive_time * 0.001);

    nsat = U1(p); p++; /* Number of SV data blocks in the record */

    for (i=0; (i < nsat) && (i < MAX_OBS); i++)
    {
        l1_snr = 0;
        l1_carrier = 0.0;
        l1_pseudorange = 0.0;
        l1_doppler = 0.0;
        l1_lli = 0;

        if (rif & M_CONCISE)
        {
            /*
            | Satellite number (1?32).
            */
            prn = U1(p);
            p++;

            /*
            | These indicate what data is loaded, is valid, etc.
            */
            flags1 = U1(p);
            p++; 
            flags2 = U1(p);
            p++;

            /*
            | These are not needed by RTKLIB.
            */
            p++;    /* I1 Satellite Elevation Angle (degrees) */
            p += 2; /* I2 Satellite Azimuth (degrees) */

            if (flags1 & 64)
            {
                /*
                | Measure of L1 signal strength (dB * 4).
                */
                l1_snr = U1(p);
                p++;
                
                /* 
                | Full L1 C/A code or P-code pseudorange (meters)
                */
                l1_pseudorange = R8(p,e);
                p += 8;
        
                /* 
                | L1 Continuous Phase (cycles).
                */
                if (flags1 & 16)
                    l1_carrier = -R8(p,e);
                p += 8;

                /*
                | L1 Doppler (Hz)
                */
                l1_doppler = R4(p,e);
                p += 4; 
            }

            if (flags1 & 1)
            {
                /*
                | Measure of L2 signal strength (dB * 4).
                */
                //l2_snr = U1(p);
                p++;
                
                /*
                | L2 Continuous Phase (cycles)
                */
                //l2_carrier = -R8(p,e);
                p += 8; 

                /*
                | L2 P-Code or L2 Encrypted Code.
                */              
                /*
								if (flags1 & 32)
                    l2_pseudorange = l1_pseudorange + R4(p,e);
                else
                    l2_carrier = -sqrt(fabs(l2_carrier));           
								*/
                p += 4;
            }
        
            /*
            | We can't use the IODE flags in this context.
            | We already have slip flags and don't need slip counters.
            */
            if (rif & M_ENHANCED)
            {
                p++; /* U1 IODE, Issue of Data Ephemeris */
                p++; /* U1 L1 cycle slip roll-over counter */ 
                p++; /* U1 L2 cycle slip roll-over counter */ 
            }           
        }
        else /* Expanded Format */
        {
            /*
            | Satellite number (1?32)
            */
            prn = U1(p);
            p++;

            /*
            | These indicate what data is loaded, is valid, etc.
            */
            flags1 = U1(p);
            p++;
            flags2 = U1(p);
            p++;

            /*
            | Indicates whether FLAGS1 bit 6 and FLAGS2 are valid.
            */
            flag_status = U1(p);
            p++;

            /*
            | These are not needed by RTKLIB.
            */
            p += 2; /* I2 Satellite Elevation Angle (degrees) */
            p += 2; /* I2 Satellite Azimuth (degrees) */

            /*
            | FLAG STATUS bit 0 set   = Bit 6 of FLAGS1 and bit 0-7 of FLAGS2 are valid.
            | FLAG STATUS bit 0 clear = Bit 6 of FLAGS1 and bit 0-7 of FLAGS2 are UNDEFINED.
            |
            | According to reference #1 above, this bit should ALWAYS be set
            | for RAWDATA. If this bit is not set, then we're lost and cannot
            | process this message any further.
            */
            if (!(flag_status & 1))
                return (OBS_ERROR);
          
            if (flags1 & 64)
            {           
                /*
                | Measure of satellite signal strength (dB).
                */
                l1_snr = R8(p,e) * 4.0;
                p += 8;

                /*
                | Full L1 C/A code or P-code pseudorange (meters).
                */
                l1_pseudorange = R8(p,e);
                p += 8;

                /*
                | L1 Continuous Phase (cycles).
                */
                if (flags1 & 16)
                    l1_carrier = -R8(p,e);
                p += 8;

                /*
                | L1 Doppler (Hz).
                */
                l1_doppler = R8(p,e);
                p += 8;

                /*
                | Reserved 8 bytes.
                */
                p += 8;
            }

            if (flags1 & 1)
            {
                /*
                | Measure of L2 signal strength (dB).
                */
                //l2_snr = R8(p,e) * 4.0;
                p += 8;

                /*
                | L2 Continuous Phase (cycles).
                */                
                //if (flags1 & 16)
                //    l2_carrier = -R8(p,e);
                p += 8;

                /*
                | L2 P-Code or L2 Encrypted Code.
                */              
                /*
								if (flags1 & 32)
                    l2_pseudorange = l1_pseudorange + R8(p,e);
                else
                    l2_carrier = -sqrt(fabs(l2_carrier));           
								*/
                p += 8;
            }   
                
            if (rif & M_ENHANCED)
            {
                /*
                | We can't use the IODE flags in this context.
                | We already have slip flags and don't need slip counters.
                */
                p++; /* U1 IODE, Issue of Data Ephemeris */
                p++; /* U1 L1 cycle slip roll-over counter */ 
                p++; /* U1 L2 cycle slip roll-over counter */ 
                p++; /* U1 Reserved byte */

                /*
                |L2 Doppler (Hz).
                */
                //l2_doppler = R8(p,e);
                p += 8;
            }
        }
        
        if (l1_pseudorange != 0.0)
        {
            if (flags1 & 2)
                l1_lli = 1;  /* L1 cycle slip */
        }

        if (!(sat=satno(SYS_GPS, prn)))
        {
        //    trace(2, "RT17: Satellite number error, PRN=%d.\n", prn);
            continue;
        }

        raw->obs.data[n].sat = sat;

        raw->obs.data[n].time    = time;
        raw->obs.data[n].L    = l1_carrier;
        raw->obs.data[n].P    = l1_pseudorange;
        raw->obs.data[n].D    = (float) l1_doppler;
        raw->obs.data[n].SNR  = (unsigned char) l1_snr;
        raw->obs.data[n].LLI  = l1_lli; 
                
        n++;
    }
        
    raw->time = time;
    raw->obs.n = n;

    return (OBS);
}


/*
| Function: decode_type_29
| Purpose:  Decode Enhanced position (record type 29)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw    = Receiver raw data control structure [Input]
|   Endian = Endianness indicator                [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit outputs:
|
|   Raw->buff[]
|   Raw->len
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   We don't actually "decode" this message per-se.
|   We're only interested in the GPS week number.
|
|   This message can only be output by receivers that can output RT27.
|   It is enabled for output by outputting positions along with RT27 data.
*/
static Error decode_type_29(raw_t *raw, int endian)
{
    unsigned char *p = raw->buff;

    if (*p < 7)
    {

		}
    else
        set_week(raw, I2(p+1, endian), (double) I4(p+3, endian));

    return (NO_ERROR1);
}

/*
| Function: get_week
| Purpose:  Get GPS week number
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw         = Pointer to raw structure [Input]
|   Receve_time = Receiver time of week    [Input]
|
| Implicit Inputs:
|
|   Raw->flag
|   Raw->week
|   Raw->receive_time
|
| Implicit Outputs:
|
|   Raw->flag
|   Raw->week
|   Raw->receive_time
|
| Return Value:
|
|   GPS week number
|
| Design Issues:
|
|   The -WEEK=n initial week option overrides everything else.
|   Week rollover and increment from the initial week and subsequent weeks is handled.
*/
static int get_week(raw_t *raw, double receive_time)
{
    int week = 0;
/*
    if (raw->flag & M_WEEK_OPTION)
    {
        if ((receive_time && raw->receive_time) &&
            (receive_time < raw->receive_time))
        {
            trace( 2, "RT17: GPS WEEK rolled over from %d to %d.\n",
                   raw->week, raw->week + 1 );

            raw->week++;
        }

        if (receive_time != 0.0)
            raw->receive_time = receive_time;
    }
    else if (!(raw->flag & M_WEEK_SCAN))
    {
        char *opt = strstr(raw->opt, "-WEEK=");

        raw->flag |= M_WEEK_SCAN;

        if (opt)
        {
            if (!sscanf(opt+6, "%d", &week) || (week <= 0))
                trace(2, "RT17: Invalid -WEEK=n receiver option value.\n");
            else
            {
                raw->week = week;
                raw->flag |= M_WEEK_OPTION;

                trace( 2, "RT17: Initial GPS WEEK explicitly set to %d "
                          "by user.\n", week );
            }
        }
    }
*/
    week = raw->week;
/*
    if (week == 0)
    {
        if ((raw->time.time == 0) && (raw->time.frac == 0.0))
            raw->time = timeget();
        
       
        time2gpst(raw->time, &week);

        if (receive_time != 0.0)
            raw->time = gpst2time(week, receive_time * 0.001);

        raw->week = week;
        

    }
 */
    return (week);
}
/*
| Function: read_i2
| Purpose:  Fetch & convert a signed two byte integer (short)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted signed two byte integer (short)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that
|   is not naturally aligned. It is then converted from the input
|   endianness to our execution platform endianness.
*/
static short read_i2(unsigned char *p, int endian) 
{
    union I2 {short i2; unsigned char c[2];} u;
    ENDIAN_TEST et;

    memcpy(&u.i2, p, sizeof(u.i2));

    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
    }
    return (u.i2);
}

/*
| Function: read_i4
| Purpose:  Fetch & convert a four byte signed integer (int)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted four byte signed integer (int)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static int read_i4(unsigned char *p, int endian)
{
    union i4 {int i4; unsigned char c[4];} u;
    ENDIAN_TEST et;

    memcpy(&u.i4, p, sizeof(u.i4));

    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
        t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
    }   
    return (u.i4);
}

/*
| Function: read_r4
| Purpose:  Fetch & convert an IEEE S_FLOAT (float)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted IEEE S_FLOAT (float)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static float read_r4(unsigned char *p, int endian)
{
    union R4 {float f; unsigned int u4;} u; 
    u.u4 = U4(p, endian);
    return (u.f);
}

/*
| Function: read_r8
| Purpose:  Fetch & convert an IEEE T_FLOAT (double)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted IEEE T_FLOAT (double)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static double read_r8(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union R8 {double d; unsigned char c[8];} u;

    memcpy(&u.d, p, sizeof(u.d));
 
    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[7]; u.c[7] = t;
        t = u.c[1]; u.c[1] = u.c[6]; u.c[6] = t;
        t = u.c[2]; u.c[2] = u.c[5]; u.c[5] = t;
        t = u.c[3]; u.c[3] = u.c[4]; u.c[4] = t;  
    }
    return (u.d);
}

/*
| Function: read_u2
| Purpose:  Fetch & convert an unsigned twe byte integer (unsigned short)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted two byte unsigned integer (unsigned short)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that
|   is not naturally aligned. It is then converted from the input
|   endianness to our execution platform endianness.
*/
static unsigned short read_u2(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union U2 {unsigned short u2; unsigned char c[2];} u;

    memcpy(&u.u2, p, sizeof(u.u2)); 
 
    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[1]; u.c[1] = t;
    }
    return (u.u2);
}

/*
| Function: read_u4
| Purpose:  Fetch & convert a four byte unsigned integer (unsigned int)
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   P      = Input pointer        [Input]
|   Endian = Endianness indicator [Input]
|
| Implicit Inputs:
|
|   <none>
|
| Implicit Outputs:
|
|   <none>
|
| Return Value:
|
|   Fetched and converted four byte unsigned integer (unsigned int)
|
| Design issues:
|
|   The data is fetched one byte at a time so as to handle data that is not
|   naturally aligned. It is then converted from the input endianness to our
|   execution platform endianness.
*/
static unsigned int read_u4(unsigned char *p, int endian)
{
    ENDIAN_TEST et;
    union U4 {unsigned int u4; unsigned char c[4];} u;

    memcpy(&u.u4, p, sizeof(u.u4));
 
    et.u2 = 0; et.c[0] = 1;  
    if ((et.u2 == 1) && (endian != LITTLE_ENDIAN))
    {
        unsigned char t;
        t = u.c[0]; u.c[0] = u.c[3]; u.c[3] = t;
        t = u.c[1]; u.c[1] = u.c[2]; u.c[2] = t;
    }   
    return (u.u4);
}



/*
| Function: set_week
| Purpose:  Set GPS week number
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw         = Pointer to raw structure [Input]
|   Week        = GPS week number          [Input]
|   Receve_time = Receiver time of week    [Input]
|
| Implicit Inputs:
|
|   Raw->week
|   Raw->flag
|
| Implicit Outputs:
|
|   Raw->week
|
| Return Value:
|
|   <none>
|
| Design Issues:
|
|   The -WEEK=n initial week option overrides us.
|
*/static void set_week(raw_t *raw, int week, double receive_time)
{
    if (!(raw->flag & M_WEEK_OPTION))
    {
        if (raw->week)
        {
            if (week != raw->week)
            {
                if (week == (raw->week + 1))
                {
									//trace(2, "RT17: GPS WEEK rolled over from %d to %d.\n", raw->week, week);
								}
                else
                {
									//trace(2, "RT17: GPS WEEK changed from %d to %d.\n", raw->week, week);
								}
            }
        }
        else
        {
					//trace(2, "RT17: GPS WEEK initially set to %d.\n", week);
				}

        raw->week = week;
    }

    /*
    | Also update the time if we can.
    */
    if (week && (receive_time != 0.0))
        raw->time = gpst2time(week, receive_time * 0.001);
}


static int sync_packet(raw_t *raw, unsigned char data)
{
    unsigned char type;

    raw->pbuff[0] = raw->pbuff[1];
    raw->pbuff[1] = raw->pbuff[2];
    raw->pbuff[2] = raw->pbuff[3];
    raw->pbuff[3] = data;

    type = raw->pbuff[2];

    /*
    | Byte 0 must be an STX character.
    | Byte 1 = status byte which we always ignore (for now).
    | Byte 2 = packet type which must be RAWDATA (57h) or RETSVDATA (55h) (for now).
    | Byte 3 = data length which must be non-zero for any packet we're interested in.
    */
    return ( (raw->pbuff[0] == STX) &&
             (data != 0) &&
             ((type == GENOUT) || (type == RAWDATA) || (type == RETSVDATA)) );
}

/*
| Function: unwrap_genout
| Purpose:  Reassemble GENOUT message by removing packet headers, trailers and page framing
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw  = Receiver raw data control structure [Input]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit Outputs:
|
|   Raw->buff[]
|   Raw->len
|   Raw->nbyte
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   The GENOUT message is broken up on _arbitrary byte boundries_ into
|   pages of no more than 246 bytes each, then wrapped with page frames,
|   packet headers and packet trailers. We reassemble the original message
|   so that it is uninterrupted by removing the extraneous packet headers,
|   trailers and page framing.
*/
static void unwrap_genout(raw_t *raw)
{
    unsigned char *p_in = raw->buff;
    unsigned char *p_out = p_in;
    unsigned int length_in, length_in_total = raw->len;
    unsigned int length_out, length_out_total = 0;

    while (length_in_total > 0)
    {
        length_in = p_in[3] + 6;
        length_out = p_in[3] - 3;
        memmove(p_out, p_in + 7, length_out);
        p_in += length_in;
        p_out += length_out;
        length_out_total += length_out;
        length_in_total -= length_in;  
    }
    raw->nbyte = raw->len = length_out_total;
}

/*
| Function: unwrap_rawdata
| Purpose:  Reassemble message by removing packet headers, trailers and page framing
| Authors:  Daniel A. Cook
|
| Formal Parameters: 
|
|   Raw  = Receiver raw data control structure [Input]
|   RIF  = Record Interpreation Flags          [Output]
|
| Implicit Inputs:
|
|   Raw->buff[]
|   Raw->len
|
| Implicit Outputs:
|
|   Raw->buff[]
|   Raw->len
|   Raw->nbyte
|
| Return Value:
|
|   -1: error message
|    0: no message (tells caller to please read more data from the stream)
|    1: input observation data
|    2: input ephemeris
|    3: input sbas message
|    9: input ion/utc parameter
|
| Design Issues:
|
|   The RAWDATA message is broken up on _arbitrary byte boundries_ into
|   pages of no more than 244 bytes each, then wrapped with page frames,
|   packet headers and packet trailers. We reassemble the original message
|   so that it is uninterrupted by removing the extraneous packet headers,
|   trailers and page framing.
|
|   While we're at it we also check to make sure the Record Interpretation
|   Flags are consistent. They should be the same in every page frame.
*/
static void unwrap_rawdata(raw_t *raw, unsigned int *rif)
{
    unsigned char *p_in = raw->buff;
    unsigned char *p_out = p_in;
    unsigned int length_in, length_in_total = raw->len;
    unsigned int length_out, length_out_total = 0;

    *rif = p_in[7];

    while (length_in_total > 0)
    {
        if ((unsigned int)p_in[7] != *rif)
        {   
					//trace( 2, "RT17: Inconsistent Record Interpretation "
          //           "Flags within a single RAWDATA message.\n" );
				}
        length_in = p_in[3] + 6;
        length_out = p_in[3] - 4;
        memmove(p_out, p_in + 8, length_out);
        p_in += length_in;
        p_out += length_out;
        length_out_total += length_out;
        length_in_total -= length_in;  
    }
    raw->nbyte = raw->len = length_out_total;
}	



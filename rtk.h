#ifndef __RTK_H
#define __RTK_H
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

//#define _DEBUG_MSG

#define SEC_PER_WEEK 604800
#define SEC_PER_HALF_WEEK 302400
#define SEC_PER_DAY 86400

#define PI          3.1415926535897932  /* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define CLIGHT      299792458.0         /* speed of light (m/s) */
#define SC2RAD      3.1415926535898     /* semi-circle to radian (IS-GPS) */
#define MUY					3.986005E14
#define OMGE        7.2921151467E-5     /* earth angular velocity (IS-GPS) (rad/s) */
#define F 					-4.442807633e-10

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

#define HION        350000.0            /* ionosphere height (m) */

#define P2_5        0.03125             /* 2^-5 */
#define P2_6        0.015625            /* 2^-6 */
#define P2_11       4.882812500000000E-04 /* 2^-11 */
#define P2_15       3.051757812500000E-05 /* 2^-15 */
#define P2_17       7.629394531250000E-06 /* 2^-17 */
#define P2_19       1.907348632812500E-06 /* 2^-19 */
#define P2_20       9.536743164062500E-07 /* 2^-20 */
#define P2_21       4.768371582031250E-07 /* 2^-21 */
#define P2_23       1.192092895507810E-07 /* 2^-23 */
#define P2_24       5.960464477539063E-08 /* 2^-24 */
#define P2_27       7.450580596923828E-09 /* 2^-27 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */
#define P2_30       9.313225746154785E-10 /* 2^-30 */
#define P2_31       4.656612873077393E-10 /* 2^-31 */
#define P2_32       2.328306436538696E-10 /* 2^-32 */
#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_35       2.910383045673370E-11 /* 2^-35 */
#define P2_38       3.637978807091710E-12 /* 2^-38 */
#define P2_39       1.818989403545856E-12 /* 2^-39 */
#define P2_40       9.094947017729280E-13 /* 2^-40 */
#define P2_43       1.136868377216160E-13 /* 2^-43 */
#define P2_48       3.552713678800501E-15 /* 2^-48 */
#define P2_50       8.881784197001252E-16 /* 2^-50 */
#define P2_55       2.775557561562891E-17 /* 2^-55 */

#define MAX_SAT 32

#define MAX_RAW_LEN 3000
#define MAX_ERRMSG 1000
#define MAX_OBS 20//max observation per epoch (rover+base)

#define MAXDTOE 7200.0
#define MAX_SOL_BUF 5
#define MAXSTRPATH  100                /* max length of stream path */
//#define MAXSTRMSG                   /* max length of stream message */
#define FREQ1       1.57542E9           /* L1/E1  frequency (Hz) */
#define LAM1	CLIGHT/FREQ1

/* solution */
//#define ENCODER
#define TIME_MEASURE

#define SOL_MSG_LEN 160
//#ifdef ENCODER
//	#define MSG_LEN 149//last byte is '\n', no '\r'
//#elif define TIME_MEASURE
//	#define MSG_LEN 146
//#else
//	#define MSG_LEN 141
//#endif

#define SOLF_ENU 
//#define SOLF_ECEF 
//#define SOLF_LLH

#define SQR(x)   ((x)*(x))

#define sos2(x) (x[0]*x[0]+x[1]*x[1])
#define sos3(x) (x[0]*x[0]+x[1]*x[1]+x[2]*x[2])
#define sos4(x) (x[0]*x[0]+x[1]*x[1]+x[2]*x[2]+x[3]*x[3])
#define norm2(x) sqrt(x[0]*x[0]+x[1]*x[1])
#define norm3(x) sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2])

#define PMODE_SINGLE 0                  /* positioning mode: single */
#define PMODE_DGPS   1                  /* positioning mode: DGPS/DGNSS */
#define PMODE_KINEMA 2                  /* positioning mode: kinematic */
#define PMODE_STATIC 3                  /* positioning mode: static */
#define PMODE_MOVEB  4                  /* positioning mode: moving-base */
#define PMODE_FIXED  5                  /* positioning mode: fixed */
#define PMODE_PPP_KINEMA 6              /* positioning mode: PPP-kinemaric */
#define PMODE_PPP_STATIC 7              /* positioning mode: PPP-static */
#define PMODE_PPP_FIXED 8               /* positioning mode: PPP-fixed */

#define SOL_TYPE_BACKWARD 0
#define SOL_TYPE_FORWARD 1
#define SOL_TYPE_COMBINED 2

#define ION_BRDC		1
#define ION_SBAS		2

#define ARMODE_OFF  0                   /* AR mode: off */
#define ARMODE_CONT 1                   /* AR mode: continuous */
#define ARMODE_INST 2                   /* AR mode: instantaneous */
#define ARMODE_FIXHOLD 3                /* AR mode: fix and hold */
#define ARMODE_PPPAR 4                  /* AR mode: PPP-AR */
#define ARMODE_PPPAR_ILS 5              /* AR mode: PPP-AR ILS */
#define ARMODE_WLNL 6                   /* AR mode: wide lane/narrow lane */
#define ARMODE_TCAR 7                   /* AR mode: triple carrier ar */

#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */ 

#define EFACT_GPS   1.0                 /* error factor: GPS */
#define EFACT_GLO   1.5                 /* error factor: GLONASS */

#define NFREQGLO    2                   /* number of carrier frequencies of GLONASS */

#define SYS_NONE    0x00                /* navigation system: none */
#define SYS_GPS     0x01                /* navigation system: GPS */
#define SYS_SBS     0x02                /* navigation system: SBAS */
#define SYS_GLO     0x04                /* navigation system: GLONASS */
#define SYS_GAL     0x08                /* navigation system: Galileo */
#define SYS_QZS     0x10                /* navigation system: QZSS */
#define SYS_CMP     0x20                /* navigation system: BeiDou */
#define SYS_LEO     0x40                /* navigation system: LEO */
#define SYS_ALL     0xFF                /* navigation system: all */

#define IONOOPT_OFF 0                   /* ionosphere option: correction off */
#define IONOOPT_BRDC 1                  /* ionosphere option: broadcast model */
#define IONOOPT_SBAS 2                  /* ionosphere option: SBAS model */
#define IONOOPT_IFLC 3                  /* ionosphere option: L1/L2 or L1/L5 iono-free LC */
#define IONOOPT_EST 4                   /* ionosphere option: estimation */
#define IONOOPT_TEC 5                   /* ionosphere option: IONEX TEC model */
#define IONOOPT_QZS 6                   /* ionosphere option: QZSS broadcast model */
#define IONOOPT_LEX 7                   /* ionosphere option: QZSS LEX ionospehre */
#define IONOOPT_STEC 8                  /* ionosphere option: SLANT TEC model */

#define TROPOPT_OFF 0                   /* troposphere option: correction off */
#define TROPOPT_SAAS 1                  /* troposphere option: Saastamoinen model */
#define TROPOPT_SBAS 2                  /* troposphere option: SBAS model */
#define TROPOPT_EST 3                   /* troposphere option: ZTD estimation */
#define TROPOPT_ESTG 4                  /* troposphere option: ZTD+grad estimation */
#define TROPOPT_COR 5                   /* troposphere option: ZTD correction */
#define TROPOPT_CORG 6                  /* troposphere option: ZTD+grad correction */

#define BASE_POS_TYPE_LLH
//#define BASE_POS_TYPE_XYZ_ECEF

#define STR_NONE     0                  /* stream type: none */
#define STR_SERIAL   1                  /* stream type: serial */
#define STR_FILE     2                  /* stream type: file */
#define STR_TCPSVR   3                  /* stream type: TCP server */
#define STR_TCPCLI   4                  /* stream type: TCP client */
#define STR_UDP      5                  /* stream type: UDP stream */
#define STR_NTRIPSVR 6                  /* stream type: NTRIP server */
#define STR_NTRIPCLI 7                  /* stream type: NTRIP client */
#define STR_FTP      8                  /* stream type: ftp */
#define STR_HTTP     9                  /* stream type: http */

#define STRFMT_RTCM2 0                  /* stream format: RTCM 2 */
#define STRFMT_RTCM3 1                  /* stream format: RTCM 3 */
#define STRFMT_OEM4  2                  /* stream format: NovAtel OEMV/4 */
#define STRFMT_OEM3  3                  /* stream format: NovAtel OEM3 */
#define STRFMT_UBX   4                  /* stream format: u-blox LEA-*T */
#define STRFMT_SS2   5                  /* stream format: NovAtel Superstar II */
#define STRFMT_CRES  6                  /* stream format: Hemisphere */
#define STRFMT_STQ   7                  /* stream format: SkyTraq S1315F */
#define STRFMT_GW10  8                  /* stream format: Furuno GW10 */
#define STRFMT_JAVAD 9                  /* stream format: JAVAD GRIL/GREIS */
#define STRFMT_NVS   10                 /* stream format: NVS NVC08C */
#define STRFMT_BINEX 11                 /* stream format: BINEX */
#define STRFMT_RT17  12                 /* stream format: Trimble RT17 */
#define STRFMT_LEXR  13                 /* stream format: Furuno LPY-10000 */
#define STRFMT_SEPT  14                 /* stream format: Septentrio */
#define STRFMT_RINEX 15                 /* stream format: RINEX */
#define STRFMT_SP3   16                 /* stream format: SP3 */
#define STRFMT_RNXCLK 17                /* stream format: RINEX CLK */
#define STRFMT_SBAS  18                 /* stream format: SBAS messages */
#define STRFMT_NMEA  19                 /* stream format: NMEA 0183 */

#define MINPRNGPS   1                   /* min satellite PRN number of GPS */
#define MAXPRNGPS   32                  /* max satellite PRN number of GPS */
#define NSATGPS     (MAXPRNGPS-MINPRNGPS+1) /* number of GPS satellites */
#define NSYSGPS     1

#ifdef ENAGLO
#define MINPRNGLO   1                   /* min satellite slot number of GLONASS */
#define MAXPRNGLO   24                  /* max satellite slot number of GLONASS */
#define NSATGLO     (MAXPRNGLO-MINPRNGLO+1) /* number of GLONASS satellites */
#define NSYSGLO     1
#else
#define MINPRNGLO   0
#define MAXPRNGLO   0
#define NSATGLO     0
#define NSYSGLO     0
#endif
#ifdef ENAGAL
#define MINPRNGAL   1                   /* min satellite PRN number of Galileo */
#define MAXPRNGAL   27                  /* max satellite PRN number of Galileo */
#define NSATGAL    (MAXPRNGAL-MINPRNGAL+1) /* number of Galileo satellites */
#define NSYSGAL     1
#else
#define MINPRNGAL   0
#define MAXPRNGAL   0
#define NSATGAL     0
#define NSYSGAL     0
#endif
#ifdef ENAQZS
#define MINPRNQZS   193                 /* min satellite PRN number of QZSS */
#define MAXPRNQZS   199                 /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                 /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189                 /* max satellite PRN number of QZSS SAIF */
#define NSATQZS     (MAXPRNQZS-MINPRNQZS+1) /* number of QZSS satellites */
#define NSYSQZS     1
#else
#define MINPRNQZS   0
#define MAXPRNQZS   0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS     0
#define NSYSQZS     0
#endif
#ifdef ENACMP
#define MINPRNCMP   1                   /* min satellite sat number of BeiDou */
#define MAXPRNCMP   35                  /* max satellite sat number of BeiDou */
#define NSATCMP     (MAXPRNCMP-MINPRNCMP+1) /* number of BeiDou satellites */
#define NSYSCMP     1
#else
#define MINPRNCMP   0
#define MAXPRNCMP   0
#define NSATCMP     0
#define NSYSCMP     0
#endif
#ifdef ENALEO
#define MINPRNLEO   1                   /* min satellite sat number of LEO */
#define MAXPRNLEO   10                  /* max satellite sat number of LEO */
#define NSATLEO     (MAXPRNLEO-MINPRNLEO+1) /* number of LEO satellites */
#define NSYSLEO     1
#else
#define MINPRNLEO   0
#define MAXPRNLEO   0
#define NSATLEO     0
#define NSYSLEO     0
#endif
#define NSYS        (NSYSGPS+NSYSGLO+NSYSGAL+NSYSQZS+NSYSCMP+NSYSLEO) /* number of systems */

#define MINPRNSBS   120                 /* min satellite PRN number of SBAS */
#define MAXPRNSBS   142                 /* max satellite PRN number of SBAS */
#define NSATSBS     (MAXPRNSBS-MINPRNSBS+1) /* number of SBAS satellites */

typedef struct{
	time_t time;	//seconds since 00:00:00 January 1st 1970
	double frac;	//fraction of second under 1s
}gtime_t;
typedef enum{
	LEN_ERROR=0,
	CHECKSUM_ERROR=1,
	INCOMPLETE=2,
	TIMING_ERROR=3,
	ID_ERROR=4,
	IODE_ERROR=5,
	TEST_ERROR=6,	
	MAT_ERR=7,
	ZERO_MAT_ERR=8,
	UNCHANGE=9,
	OBS_ERROR=0x0A,
	EPH_ERROR=0x0B,	
	ID4_ERROR=0x0C,
	ID5_ERROR=0x0D,
	NO_ERROR1=0x0E,
	NO_ERROR2=0x0F,
	NO_ERROR3=0x10,
	EPHEMERIS=0x11,
	OBS=0x12,
	SOLUTION=0x13,
}Error;
typedef struct {        /* SNR mask type */
    int ena[2];         /* enable flag {rover,base} */
    double mask[9]; /* mask (dBHz) at 5,15,25,35,45,55,65,75,85 deg */
} snrmask_t;
typedef struct{
	uint8_t sat; //satellite number
	int iode, iodc;
	int sva;	//SV accuracy
	int svh;	//SV health (0: ok)
	int week;	//gps week
	gtime_t toe, toc, ttr;	//Toe, Toc, T transmit
	double A, e, i0, OMG0, omg, M0, deln, OMgd, idot;
	double crc, crs, cuc, cus, cic, cis;
	double toes;//Toe(s) in week
	double fit;//fit interval
	double f0, f1, f2;//SV clock params
	double tgd[4];	//group delay params
}eph_t;
typedef struct{//observation data record, single band
	gtime_t time; //receiver sampling time
	uint8_t sat, rcv;//satellite/receiver number
	uint8_t SNR;
	uint8_t LLI;//lost of clock indicator

	double L;
	double P;
	float D;	
}obsd_t;
typedef struct{
	int n, nmax;	//number of observation data/allocated
	obsd_t data[MAX_SAT];//~280 bytes
}obs_t;
typedef struct {        /* earth rotation parameter data type */
    double mjd;         /* mjd (days) */
    double xp,yp;       /* pole offset (rad) */
    double xpr,ypr;     /* pole offset rate (rad/day) */
    double ut1_utc;     /* ut1-utc (s) */
    double lod;         /* length of day (s/day) */
} erpd_t;
typedef struct {        /* earth rotation parameter type */
    int n,nmax;         /* number and max number of data */
    erpd_t *data;       /* earth rotation parameter data */
} erp_t;

typedef struct {        /* processing options type */
    int mode;           /* positioning mode (PMODE_???) */
    int soltype;        /* solution type (0:forward,1:backward,2:combined) */
    int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    int navsys;         /* navigation system */
    double elmin;       /* elevation mask angle (rad) */
    snrmask_t snrmask;  /* SNR mask */
    int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
    int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    int maxout;         /* obs outage count to reset bias */
    int minlock;        /* min lock count to fix ambiguity */
    int minfix;         /* min fix count to hold ambiguity */
    int ionoopt;        /* ionosphere option (IONOOPT_???) */
    int tropopt;        /* troposphere option (TROPOPT_???) */
    int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
    int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
    int niter;          /* number of filter iteration */
    int codesmooth;     /* code smoothing window size (0:none) */
    int intpref;        /* interpolate reference obs (for post mission) */
    int sbascorr;       /* SBAS correction options */
    int sbassatsel;     /* SBAS satellite selection (0:all) */
    int rovpos;         /* rover position for fixed mode */
    int refpos;         /* base position for relative mode */
                        /* (0:pos in prcopt,  1:average of single pos, */
                        /*  2:read from file, 3:rinex header, 4:rtcm pos) */
    double eratio; /* code/phase error ratio */
    double err[5];      /* measurement error factor */
                        /* [0]:reserved */
                        /* [1-3]:error factor a/b/c of phase (m) */
                        /* [4]:doppler frequency (hz) */
    double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
    double prn[5];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv */
    double sclkstab;    /* satellite clock stability (sec/sec) */
    double thresar[4];  /* AR validation threshold */
    double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
    double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
    double thresslip;   /* slip threshold of geometry-free phase (m) */
    double maxtdiff;    /* max difference of time (sec) */
    double maxinno;     /* reject threshold of innovation (m) */
    double maxgdop;     /* reject threshold of gdop */
    double baseline[2]; /* baseline length constraint {const,sigma} (m) */
    double ru[3];       /* rover position for fixed mode {x,y,z} (ecef) (m) */
    double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
//    char anttype[2][MAXANT]; /* antenna types {rover,base} */
//    double antdel[2][3]; /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
//    pcv_t pcvr[2];      /* receiver antenna parameters {rov,base} */
//    unsigned char exsats[MAXSAT]; /* excluded satellites (1:excluded,2:included) */
//    char rnxopt[2][256]; /* rinex options {rover,base} */
//    int  posopt[6];     /* positioning options */
//    int  syncsol;       /* solution sync mode (0:off,1:on) */
//    double odisp[2][6*11]; /* ocean tide loading parameters {rov,base} */
//    exterr_t exterr;    /* extended receiver error model */
} prcopt_t;
extern const prcopt_t default_opt;
typedef struct {        /* almanac type */
    int sat;            /* satellite number */
    int svh;            /* sv health (0:ok) */
    int svconf;         /* as and sv config */
    int week;           /* GPS/QZS: gps week, GAL: galileo week */
    gtime_t toa;        /* Toa */
                        /* SV orbit parameters */
    double A,e,i0,OMG0,omg,M0,OMGd;
    double toas;        /* Toa (s) in week */
    double f0,f1;       /* SV clock parameters (af0,af1) */
} alm_t;
typedef struct{
	int n,nmax;//number of broadcast ephemeris
//	int na, namax;//number of almanac data
	eph_t* eph;
//	erp_t erp;//earth rotation param
//	alm_t *alm;         /* almanac data */
	double utc_gps[4];//GPS delta-UTC params {A0, A1, T, W}
	double ion_gps[8];//GPS iono model params {a0-a3,b0-b3}
	int leaps;
//	double lam[MAX_SAT];//carrier wave length
//	double cbias[MAX_SAT][3];//code bias (0:p1-p2,1:p1-c1,2:p2-c2)
}nav_t;
typedef struct
{
	gtime_t time; //message time
	gtime_t tobs;	//observation data time	
	obs_t obs;	//observation data	
//	obs_t obuf;	//observation data buffer
	nav_t nav;
	int ephsat;	//sat number of update ephemeris (0: no satellite)
	uint8_t subfrm[MAX_SAT][150]; // ~5Kb
	double lockt[MAX_SAT][2];
	double icpp[MAX_SAT], off[MAX_SAT], icpc;
	int nbyte;
	int nbyteLeft;
	int len;
	int iod;	//issue of data
	int tod;	//time of day (ms)
	int flag; //general purpose flag
	uint8_t buff[MAX_RAW_LEN];		
}raw_t;
typedef struct{
	gtime_t time;
	double rr[6];//pos/vel (m,m/s)
	float qr[6];//pos variance/covariance (m^2)
	                        /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                        /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
	double dtr[1];//receiver clock bias
	uint8_t type;//0: xyz-ecef, 1:enu-baseline
	uint8_t stat;//solution status
	uint8_t ns;//number of valid satellites
	float age;//age of differential (s)
	float ratio;//for validation
	int processTime;
	int encoder;
	char *result;
}sol_t;
typedef struct{
	uint8_t vs;//valid sat flag
	double azel[2];//azimuth, elevation angle (rad)
	double resp;//residuals of pseudorange (m)
	double resc;//residulas of carrier-phase (m)
	uint8_t vsat;//valid sat flag
	uint8_t snr;//signal strength (0.25dBHz)
	uint8_t fix;//1:fix,2:float,3:hold
	uint8_t slip;//cycle-slip flag
	unsigned int lock;//lock counter of phase
	unsigned int outc;//obs outage counter of phase
	unsigned int slipc;//cycle-slip counter
	unsigned int rejc;//reject counter
	double phw;//phase windup
	gtime_t pt[2];//previous carrier-phase time
	double ph[2];//previous carrier-phase observable (cycle)
}ssat_t;
typedef struct{
	gtime_t epoch[4];//last epoch
	int fixcnt;//fix counter
	char flags[MAX_SAT];//fix flags
	double n[4];//number of epoch
	double LC[4];//linear combination average
	double LCv[4];//linear combination variance
}ambc_t;
typedef struct{
	sol_t sol;
	double rb[6];//base position/velocity (ecef) (m,m/s)
	int nx, na;//number of float states/fixed states
	double tt;//time difference between current and previous
	double x[35], P[35*35];//float states and their cov
	//float state:pos(3),vel(3),acc(3),sat(32)
	double xa[3], Pa[9];//fixed stated and their cov
	//fix state:pos(3),vel(3),acc(3)
	int nfix;//number of continuous fixes of ambiguity
	//ambc_t ambc[MAX_SAT];
	ssat_t ssat[MAX_SAT];
	int neb;//bytes in error msg buffer
	char errbuf[MAX_ERRMSG];//error msg buffer
	int errLen;
	prcopt_t opt;
}rtk_t;
typedef struct {        /* stream type */
    int type;           /* type (STR_???) */
    int mode;           /* mode (STR_MODE_?) */
    int state;          /* state (-1:error,0:close,1:open) */
    //unsigned int inb,inr;   /* input bytes/rate */
    //unsigned int outb,outr; /* output bytes/rate */
    //unsigned int tick,tact; /* tick/active tick */
    //unsigned int inbt,outbt; /* input/output bytes at tick */
    //lock_t lock;        /* lock flag */
    //void *port;         /* type dependent port control struct */
    char path[MAXSTRPATH]; /* stream path */
    //char msg [MAXSTRMSG];  /* stream message */
} stream_t;
typedef struct{
	rtk_t rtk;
	int nb[2];
	int buffPtr[2];//buffer pointer
	uint8_t* buff[2];//input buffers 
//	uint8_t sbuff[100];//solution buffer
//	sol_t solbuf;
	raw_t raw[2];
	gtime_t ftime;//download time
	obs_t obs[2];//observation data (rover,base)
	nav_t nav;
	int format[3];      /* input format {rov,base,corr} */
}rtksvr_t;
//rtkcmn.c
extern const double chisqr[100];
gtime_t epoch2time(const double* ep);
gtime_t utc2gpst(gtime_t t);
gtime_t gpst2utc(gtime_t t);
gtime_t gpst2time(int week, double sec);
double time2gpst(gtime_t t, int* week);
uint32_t getbitu(const uint8_t*buff,int word, int pos, int len);
int32_t getbits(const uint8_t*buff,int word, int pos, int len);
void setbit(uint8_t*buff,int word, int pos, int len, int32_t value);
double timediff(gtime_t t1, gtime_t t2);
gtime_t timeadd(gtime_t t, double sec);
extern double *mat(int r, int c);
extern double *zeros(int r, int c);
extern void matcpy(double *A, const double *B, int n, int m);
extern int *imat(int n, int m);
extern void matmul(const char *tr, int n, int k, int m, double alpha,
                   const double *A, const double *B, double beta, double *C);
extern int matinv(double *A, int n);
extern int solve(const char *tr, const double *A, const double *Y, int n,
                 int m, double *X);
extern int lsq(const double *A,const double *y,int n, int m, double *x, double *Q);
extern double norm(const double *a, int n);
extern void dops(int ns, const double *azel, double elmin, double *dop);
extern void xyz2enu(const double *pos, double *E);
extern void ecef2enu(const double *pos, const double *r, double *e);
extern double satazel(const double *pos, const double *e, double *azel);
extern void ecef2pos(const double *r, double *pos);
extern void pos2ecef(const double *pos, double *r);
extern double geodist(const double *rs, const double *rr, double *e);
extern double tropmodel(gtime_t time, const double *pos, const double *azel,
                        double humi);
extern double ionmodel(gtime_t t, const double *ion, const double *pos,
                       const double *azel);
extern double dot(const double *a, const double *b, int n);
extern void time2epoch(gtime_t t, double *ep);
extern void covenu(const double *pos, const double *P, double *Q);
extern double tropmapf(gtime_t time, const double pos[], const double azel[],
                       double *mapfw);
extern int filter(double *x, double *P, const double *H, const double *v,
                  const double *R, int n, int m);
extern double *eye(int n);
extern int testsnr(int base, double el, double snr,
                   const snrmask_t *mask);
extern int satno(int sys, int prn);
//rcvraw.c
void init_raw(raw_t *raw,eph_t *eph);
extern Error decode_subfrm1(const uint8_t* buff,eph_t *eph);
extern Error decode_subfrm2(const uint8_t* buff,eph_t *eph);
extern Error decode_subfrm3(const uint8_t* buff,eph_t *eph);
extern Error decode_frame(const unsigned char *buff, eph_t *eph,
                        double *ion, double *utc, int *leaps);
//rtksvr.c
extern void rtksvrstart(rtksvr_t* svr);
extern void updatesvr(rtksvr_t* svr,Error Err, int index);
//rtkpos.c
extern void rtkinit(rtk_t *rtk,const prcopt_t *opt);
extern int rtkpos(rtk_t *rtk,const obsd_t *obs, int n, const nav_t *nav);
//pntpos.c
extern int pntpos(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol,
	double *azel, ssat_t *ssat, const prcopt_t *opt,char **msg);
//ephemeris.c
extern void satposs(gtime_t teph,const obsd_t *obs,int n,const nav_t *nav,
		double *rs, double *dts, double *var, int *svh,char **msg);
//solution.c
extern void outsol(const sol_t *sol, const double *rb);
//SuperStarII.c
extern Error input_ss2(raw_t* raw, uint8_t data);
//ublox
extern Error input_ubx(raw_t* raw, uint8_t data);
//lambda.c
extern int lambda(int n, int m, const double *a, const double *Q, double *f,
                  double *s);
#endif 

#include "rtk.h"
#include "main.h"

#define UBXSYNC1    0xB5        /* ubx message sync code 1 */
#define UBXSYNC2    0x62        /* ubx message sync code 2 */
#define UBXCFG      0x06        /* ubx message cfg-??? */

#define ID_NAVSOL   0x0106      /* ubx message id: nav solution info */
#define ID_NAVTIME  0x0120      /* ubx message id: nav time gps */
#define ID_RXMRAW   0x0210      /* ubx message id: raw measurement data */
#define ID_RXMSFRB  0x0211      /* ubx message id: subframe buffer */
#define ID_RXMSFRBX 0x0213      /* ubx message id: raw subframe data */
#define ID_RXMRAWX  0x0215      /* ubx message id: multi-gnss raw meas data */
#define ID_TRKD5    0x030A      /* ubx message id: trace mesurement data */
#define ID_TRKMEAS  0x0310      /* ubx message id: trace mesurement data */
#define ID_TRKSFRBX 0x030F      /* ubx message id: trace subframe buffer */

#define FU1         1           /* ubx message field types */
#define FU2         2
#define FU4         3
#define FI1         4
#define FI2         5
#define FI4         6
#define FR4         7
#define FR8         8
#define FS32        9

#define P2_10       0.0009765625 /* 2^-10 */

#define ROUND(x)    (int)floor((x)+0.5)

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((char *)(p)))
static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
static int            I4(unsigned char *p) {int            u; memcpy(&u,p,4); return u;}
static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}

static double         I8(unsigned char *p) {return I4(p+4)*4294967296.0+U4(p);}

/* set fields (little-endian) ------------------------------------------------*/
static void setU1(unsigned char *p, unsigned char  u) {*p=u;}
static void setU2(unsigned char *p, unsigned short u) {memcpy(p,&u,2);}
static void setU4(unsigned char *p, unsigned int   u) {memcpy(p,&u,4);}
static void setI1(unsigned char *p, char           i) {*p=(unsigned char)i;}
static void setI2(unsigned char *p, short          i) {memcpy(p,&i,2);}
static void setI4(unsigned char *p, int            i) {memcpy(p,&i,4);}
static void setR4(unsigned char *p, float          r) {memcpy(p,&r,4);}
static void setR8(unsigned char *p, double         r) {memcpy(p,&r,8);}


/* decode ubx-nav-sol: navigation solution -----------------------------------*/
static Error decode_navsol(raw_t *raw)
{
    int itow,ftow,week;
    unsigned char *p=raw->buff+6;
       
    itow=U4(p);
    ftow=I4(p+4);
    week=U2(p+8);
    if ((U1(p+11)&0x0C)==0x0C) {
        raw->time=gpst2time(week,itow*1E-3+ftow*1E-9);
    }
    return SOLUTION;
}
/* ubx gnss indicator (ref [2] 25) -------------------------------------------*/
static int ubx_sys(int ind)
{
    switch (ind) {
        case 0: return SYS_GPS;
        case 1: return SYS_SBS;
        case 2: return SYS_GAL;
        case 3: return SYS_CMP;
        case 5: return SYS_QZS;
        case 6: return SYS_GLO;
    }
    return 0;
}
/* decode ubx-trk-meas: trace measurement data -------------------------------*/
static Error decode_trkmeas(raw_t *raw)
{
    //static double adrs[MAX_SAT]={0};
    gtime_t time;
    double ts,tr=-1.0,t,tau,snr,adr,dop,lock2;
    int i,n=0,nch,sys,prn,sat,qi,flag,week;
    unsigned char *p=raw->buff+6;

    if (!raw->time.time) return TIMING_ERROR;
    
    /* number of channels */
    nch=U1(p+2);
    
    if (raw->len<112+nch*56) {
       return LEN_ERROR;
    }
    /* time-tag = max(transmission time + 0.08) rounded by 100 ms */
    for (i=0,p=raw->buff+110;i<nch;i++,p+=56) {
        if (U1(p+1)<4||ubx_sys(U1(p+4))!=SYS_GPS) continue;
        if ((t=I8(p+24)*P2_32/1000.0)>tr) tr=t;
    }
    if (tr<0.0) return TIMING_ERROR;
    
    tr=ROUND((tr+0.08)/0.1)*0.1;
    
    /* adjust week handover */
    t=time2gpst(raw->time,&week);
    if      (tr<t-302400.0) week--;
    else if (tr>t+302400.0) week++;
    time=gpst2time(week,tr);
    
    //utc_gpst=timediff(gpst2utc(time),time);
    
    for (i=0,p=raw->buff+110;i<nch;i++,p+=56) {
        
        /* quality indicator (0:idle,1:search,2:aquired,3:unusable, */
        /*                    4:code lock,5,6,7:code/carrier lock) */
        qi=U1(p+1);
        if (qi<4||7<qi) continue;
        
        /* system and satellite number */
        if (!(sys=ubx_sys(U1(p+4)))) {
         //   trace(2,"ubx trkmeas: system error\n");
            continue;
        }
        prn=U1(p+5)+(sys==SYS_QZS?192:0);
        if (!(sat=satno(sys,prn))) {
         //   trace(2,"ubx trkmeas sat number error: sys=%2d prn=%2d\n",sys,prn);
            continue;
        }
        /* transmission time */
        ts=I8(p+24)*P2_32/1000.0;
        //if      (sys==SYS_CMP) ts+=14.0;             /* bdt  -> gpst */
        //else if (sys==SYS_GLO) ts-=10800.0+utc_gpst; /* glot -> gpst */
        
        /* signal travel time */
        tau=tr-ts;
        if      (tau<-302400.0) tau+=604800.0;
        else if (tau> 302400.0) tau-=604800.0;
        
        //frq  =U1(p+ 7)-7; /* frequency */
        flag =U1(p+ 8);   /* tracking status */
        //lock1=U1(p+16);   /* code lock count */
        lock2=U1(p+17);   /* phase lock count */
        snr  =U2(p+20)/256.0;
        adr  =I8(p+32)*P2_32+(flag&0x40?0.5:0.0);
        dop  =I4(p+40)*P2_10*10.0;
        
        /* set slip flag */
        if (lock2==0||lock2<raw->lockt[sat-1][0]) raw->lockt[sat-1][1]=1.0;
        raw->lockt[sat-1][0]=lock2;
        
#if 0 /* for debug */
        trace(2,"[%2d] qi=%d sys=%d prn=%3d frq=%2d flag=%02X ?=%02X %02X "
              "%02X %02X %02X %02X %02X lock=%3d %3d ts=%10.3f snr=%4.1f "
              "dop=%9.3f adr=%13.3f %6.3f\n",U1(p),qi,U1(p+4),prn,frq,flag,
              U1(p+9),U1(p+10),U1(p+11),U1(p+12),U1(p+13),U1(p+14),U1(p+15),
              lock1,lock2,ts,snr,dop,adr,
              adrs[sat-1]==0.0||dop==0.0?0.0:(adr-adrs[sat-1])-dop);
#endif
        //adrs[sat-1]=adr;
        
        /* check phase lock */
        if (!(flag&0x20)) continue;
        
        raw->obs.data[n].time=time;
        raw->obs.data[n].sat=sat;
        raw->obs.data[n].P=tau*CLIGHT;
        raw->obs.data[n].L=-adr;
        raw->obs.data[n].D=(float)dop;
        raw->obs.data[n].SNR=(unsigned char)(snr*4.0);
        //raw->obs.data[n].code=sys==SYS_CMP?CODE_L1I:CODE_L1C;
        raw->obs.data[n].LLI=raw->lockt[sat-1][1]>0.0?1:0;
        raw->lockt[sat-1][1]=0.0;
        
//        for (j=1;j<NFREQ+NEXOBS;j++) {
//            raw->obs.data[n].L[j]=raw->obs.data[n].P[j]=0.0;
//            raw->obs.data[n].D[j]=0.0;
//            raw->obs.data[n].SNR[j]=raw->obs.data[n].LLI[j]=0;
//            raw->obs.data[n].code[j]=CODE_NONE;
//        }
        n++;
    }
    if (n<=0) return OBS_ERROR;
    raw->time=time;
    raw->obs.n=n;
    return OBS;
}
/* decode ephemeris ----------------------------------------------------------*/
static Error decode_ephem(int sat, raw_t *raw)
{
  Error error;  
	eph_t eph={0};
  
	if (raw->time.time == 0)//time tu receiver do neu
		//dung gps tow thi bi week rollover
		return TIMING_ERROR;
	//eph.ttr = raw->time;
	time2gpst(raw->time,&eph.week);	
	
	error = decode_subfrm1(raw->subfrm[sat-1],&eph);
	if (error != NO_ERROR2)
		return error;
	error = decode_subfrm2(raw->subfrm[sat-1]+30,&eph);
	if (error != NO_ERROR2)
		return error;
	error = decode_subfrm3(raw->subfrm[sat-1]+60,&eph);
	if (error != NO_ERROR2)
		return error;	
        
  if (eph.iode==raw->nav.eph[sat-1].iode&&
            eph.iodc==raw->nav.eph[sat-1].iodc) 
		return UNCHANGE;
    
  eph.sat=sat;
	//SendIntStr(sat);
	//eph.ttr = raw->time;
  raw->nav.eph[sat-1]=eph;
  raw->ephsat=sat;
  return EPHEMERIS;
}
/* decode almanac and ion/utc ------------------------------------------------*/
static int decode_alm1(int sat, raw_t *raw)
{
  return decode_frame(raw->subfrm[sat-1]+90,NULL,raw->nav.alm,raw->nav.ion_gps,
                     raw->nav.utc_gps,&raw->nav.leaps);
}
/* decode almanac ------------------------------------------------------------*/
static Error decode_alm2(int sat, raw_t *raw)
{
  return decode_frame(raw->subfrm[sat-1]+120,NULL,raw->nav.alm,NULL,NULL,NULL);
}
/* decode gps and qzss navigation data ---------------------------------------*/
static Error decode_nav(raw_t *raw, int sat, int off)
{
    unsigned int words[10];
    int i,id;
    unsigned char *p=raw->buff+6+off;
    
    if (raw->len<48+off) {
        //trace(2,"ubx rawsfrbx length error: sat=%d len=%d\n",sat,raw->len);
        return LEN_ERROR;
    }
    for (i=0;i<10;i++,p+=4) words[i]=U4(p)>>6; /* 24 bits without parity */
    
    id=(words[1]>>2)&7;
    if (id<1||5<id) {
        //trace(2,"ubx rawsfrbx subfrm id error: sat=%2d\n",sat);
      return ID_ERROR;
			
    }
    for (i=0;i<10;i++) {
        setbit(raw->subfrm[sat-1]+(id-1)*30,i+1,0,24,words[i]);
    }
    //SendIntStr(id);
    if (id==3) 
		{
			Error err = decode_ephem(sat,raw);
			//SendIntStr(err);
			return err;
		}
		if (id==4) return ID4_ERROR;
		if (id==5) return ID5_ERROR;
//    if (id==4) return decode_alm1 (sat,raw);
//    if (id==5) return decode_alm2 (sat,raw);

	return NO_ERROR1;
}

/* decode ubx-trk-sfrbx: subframe buffer extension ---------------------------*/
Error decode_trksfrbx(raw_t *raw)
{
    int prn,sat,sys;
    unsigned char *p=raw->buff+6;
 
    if (!(sys=ubx_sys(U1(p+1)))) {
        return EPH_ERROR;
    }
    prn=U1(p+2)+(sys==SYS_QZS?192:0);//*************************
    if (!(sat=satno(sys,prn))) {
//        trace(2,"ubx trksfrbx sat number error: sys=%d prn=%d\n",sys,prn);
        return EPH_ERROR;
		}

    return decode_nav(raw,sat,13);    
}
/* checksum ------------------------------------------------------------------*/
static int checksum(unsigned char *buff, int len)
{
    unsigned char cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}
/* decode ublox raw message --------------------------------------------------*/
static Error decode_ubx(raw_t *raw)
{
  Error err;  
	int type=(U1(raw->buff+2)<<8)+U1(raw->buff+3);
      
    /* checksum */
    if (!checksum(raw->buff,raw->len)) {
        
        return CHECKSUM_ERROR;
    }
    switch (type) {
        //case ID_RXMRAW  : return decode_rxmraw  (raw);
        //case ID_RXMRAWX : return decode_rxmrawx (raw);
        //case ID_RXMSFRB : return decode_rxmsfrb (raw);
        //case ID_RXMSFRBX: return decode_rxmsfrbx(raw);
        case ID_NAVSOL  : 
					//return SOLUTION;
					return decode_navsol(raw);
        //case ID_NAVTIME : return decode_navtime (raw);
        case ID_TRKMEAS :
					//return OBS;	
					return decode_trkmeas (raw);
        //case ID_TRKD5   : return decode_trkd5   (raw);
        case ID_TRKSFRBX: 
					//return EPHEMERIS;
					err=decode_trksfrbx(raw);
	//				SendIntStr(err);
					return err;
    }

    return NO_ERROR3;
}
/* sync code -----------------------------------------------------------------*/
static int sync_ubx(unsigned char *buff, unsigned char data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==UBXSYNC1&&buff[1]==UBXSYNC2;
}
Error input_ubx(raw_t* raw, uint8_t data)
{
		if ((raw->nbyte) == 0)
		{
			if (!sync_ubx(raw->buff,data))
				return INCOMPLETE;
			raw->nbyte = 2;
			return INCOMPLETE;		
		}
		raw->buff[raw->nbyte++] = data;
		if (raw->nbyte < 6)
			return INCOMPLETE;
		else if (raw->nbyte == 6)
		{			
			raw->len = U2(raw->buff+4)+8;
		}
		if (raw->nbyte == raw->len)
		{
			raw->nbyte = 0;
			return decode_ubx(raw);
		}
		return INCOMPLETE;
	
}
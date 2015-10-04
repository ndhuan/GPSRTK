#include <cstdlib>
#include "rtk.h"
/* -- void init_raw(raw_t* raw) --------------------------------------
 * 
 * Description	: 
 * Parameters	: 
 * Return		: 
 */
void init_raw(raw_t* raw,eph_t *eph,alm_t* alm)
{
	int i=0,j=0;	
	gtime_t time0 = {0};
	obsd_t data0 = {{0}};
	alm_t  alm0 ={0,-1};
	eph_t eph0 ={0,-1,-1};
	
	raw->time = raw->tobs = time0;
	
	raw->obs.n=0;
//	raw->obuf.n=0;
	for (i=0;i<MAX_OBS;i++)
	{
		raw->obs.data[i]=data0;
	}

	for (i=0;i<MAX_SAT;i++)
	{
		eph[i]=eph0;
		alm[i]=alm0;
	}
	raw->nav.eph = eph;
	raw->nav.alm = alm;
	raw->nav.n = MAX_SAT;
	raw->ephsat = 0;
	
	for (i=0;i<MAX_SAT;i++)
	{
		for (j=0;j<150;j++)
		{
			raw->subfrm[i][j] = 0;			
		}
		raw->icpp[i]=raw->off[i]=0;		
	}
	raw->icpc=0;

	raw->nbyte = raw->len = 0;
	raw->iod = raw->flag = 0;
	raw->tod = -1;

	for (i=0;i<MAX_RAW_LEN;i++)
		raw->buff[i] = 0;
	
}
Error decode_subfrm1(const uint8_t* buff,eph_t *eph)
{
	double tow,toc;
	int week, iodc0, iodc1;
	
	tow = getbitu(buff,2,0,17)*6.0;
//	eph->week = getbitu(buff,3,0,10);
	
	eph->sva = getbitu(buff,3,12,4);
	eph->svh = getbitu(buff,3,16,6);
	iodc0 = getbitu(buff,3,22,2);
	eph->tgd[0] = getbits(buff,7,16,8)*P2_31;
	iodc1 = getbitu(buff,8,0,8);
	toc = getbitu(buff,8,8,16)*16.0;
	eph->f2 = getbits(buff,9,0,8)*P2_55;
	eph->f1 = getbits(buff,9,8,16)*P2_43;
	eph->f0 = getbits(buff,10,0,22)*P2_31;
	
	eph->iodc = (iodc0<<8)+iodc1;
	
	// eph->week=adjgpsweek(week); /* week of tow */ 
	// using raw->time to adjust eph->week instead
	eph->ttr = gpst2time(eph->week,tow);
	eph->toc = gpst2time(eph->week,toc);
	
	return NO_ERROR2;
}
Error decode_subfrm2(const uint8_t* buff,eph_t *eph)
{
	double sqrtA;
	eph->iode = getbitu(buff,3,0,8);
	eph->crs = getbits(buff,3,8,16)*P2_5;
	eph->deln = getbits(buff,4,0,16)*P2_43*SC2RAD;
	eph->M0 = getbits(buff,4,16,32)*P2_31*SC2RAD;
	eph->cuc = getbits(buff,6,0,16)*P2_29;
	eph->e = getbitu(buff,6,16,32)*P2_33;
	eph->cus = getbits(buff,8,0,16)*P2_29;
	sqrtA = getbitu(buff,8,16,32)*P2_19;
	eph->A=sqrtA*sqrtA;
	eph->toes = getbitu(buff,10,0,16)*16.0;
	eph->fit =getbitu(buff,10,16,1)?0.0:4.0;
	
	return NO_ERROR2;
}
Error decode_subfrm3(const uint8_t* buff,eph_t *eph)
{
	uint32_t iode;
	double tow;
	double toc;
	
	eph->cic = getbits(buff,3,0,16)*P2_29;
	eph->OMG0 = getbits(buff,3,16,32)*P2_31*SC2RAD;
	eph->cis = getbits(buff,5,0,16)*P2_29;
	eph->i0 = getbits(buff,5,16,32)*P2_31*SC2RAD;
	eph->crc = getbits(buff,7,0,16)*P2_5;
	eph->omg = getbits(buff,7,16,32)*P2_31*SC2RAD;
	eph->OMgd = getbits(buff,9,0,24)*P2_43*SC2RAD;
	iode = getbitu(buff,10,0,8);
	eph->idot = getbits(buff,10,8,14)*P2_43*SC2RAD;
	
	//iode in subframe 3 must be the same as one in subfrm1 and 8 bit LSBs of iodc (20.3.4.4 P152)
	if ((iode != eph->iode) || (iode != (eph->iodc & 0xff)))
		return IODE_ERROR;
	//correct gps time (P98)
	tow = time2gpst(eph->ttr,&eph->week);
	toc=time2gpst(eph->toc,NULL);
  if      (eph->toes<tow-302400.0) {eph->week++; tow-=604800.0;}
  else if (eph->toes>tow+302400.0) {eph->week--; tow+=604800.0;}
  eph->toe=gpst2time(eph->week,eph->toes);
  eph->toc=gpst2time(eph->week,toc);
  eph->ttr=gpst2time(eph->week,tow);
	return NO_ERROR2;
}
///* decode gps navigation data subframe 4 -------------------------------------*/
//static void decode_gps_subfrm4(const unsigned char *buff, alm_t *alm,
//                               double *ion, double *utc, int *leaps)
//{
//    int i,sat,svid=getbitu(buff,50,6);
//    
//    if (25<=svid&&svid<=32) { /* page 2,3,4,5,7,8,9,10 */
//        
//        /* decode almanac */
//        sat=getbitu(buff,50,6);
//        if (1<=sat&&sat<=32) decode_almanac(buff,sat,alm);
//    }
//    else if (svid==63) { /* page 25 */
//        
//        /* decode as and sv config */
//        i=56;
//        for (sat=1;sat<=32;sat++) {
//            if (alm) alm[sat-1].svconf=getbitu(buff,i,4); i+=4;
//        }
//        /* decode sv health */
//        i=186;
//        for (sat=25;sat<=32;sat++) {
//            if (alm) alm[sat-1].svh   =getbitu(buff,i,6); i+=6;
//        }
//    }
//    else if (svid==56) { /* page 18 */
//        
//        /* decode ion/utc parameters */
//        if (ion) {
//            i=56;
//            ion[0]=getbits(buff,i, 8)*P2_30;     i+= 8;
//            ion[1]=getbits(buff,i, 8)*P2_27;     i+= 8;
//            ion[2]=getbits(buff,i, 8)*P2_24;     i+= 8;
//            ion[3]=getbits(buff,i, 8)*P2_24;     i+= 8;
//            ion[4]=getbits(buff,i, 8)*pow(2,11); i+= 8;
//            ion[5]=getbits(buff,i, 8)*pow(2,14); i+= 8;
//            ion[6]=getbits(buff,i, 8)*pow(2,16); i+= 8;
//            ion[7]=getbits(buff,i, 8)*pow(2,16);
//        }
//        if (utc) {
//            i=120;
//            utc[1]=getbits(buff,i,24)*P2_50;     i+=24;
//            utc[0]=getbits(buff,i,32)*P2_30;     i+=32;
//            utc[2]=getbits(buff,i, 8)*pow(2,12); i+= 8;
//            utc[3]=getbitu(buff,i, 8);
//        }
//        if (leaps) {
//            i=192;
//            *leaps=getbits(buff,i,8);
//        }
//    }
//		return NO_ERROR;
//}
///* decode gps navigation data subframe 5 -------------------------------------*/
//static void decode_gps_subfrm5(const unsigned char *buff, alm_t *alm)
//{
//    double toas;
//    int i,sat,week,svid=getbitu(buff,50,6);
//    
//    if (1<=svid&&svid<=24) { /* page 1-24 */
//        
//        /* decode almanac */
//        sat=getbitu(buff,50,6);
//        if (1<=sat&&sat<=32) decode_almanac(buff,sat,alm);
//    }
//    else if (svid==51) { /* page 25 */
//        
//        if (alm) {
//            i=56;
//            toas=getbitu(buff,i,8)*4096; i+=8;
//            week=getbitu(buff,i,8);      i+=8;
//            week=adjgpsweek(week);
//            
//            /* decode sv health */
//            for (sat=1;sat<=24;sat++) {
//                alm[sat-1].svh=getbitu(buff,i,6); i+=6;
//            }
//            for (sat=1;sat<=32;sat++) {
//                alm[sat-1].toas=toas;
//                alm[sat-1].week=week;
//                alm[sat-1].toa=gpst2time(week,toas);
//            }
//        }
//    }
//		return NO_ERROR;
//}
extern Error decode_frame(const unsigned char *buff, eph_t *eph, alm_t *alm,
                        double *ion, double *utc, int *leaps)
{
	int32_t id=getbitu(buff,2,20,3);
  switch (id) {
				case 1: return decode_subfrm1(buff,eph);
        case 2: return decode_subfrm2(buff,eph);
        case 3: return decode_subfrm3(buff,eph);
//        case 4: return decode_subfrm4(buff,alm,ion,utc,leaps);
//        case 5: return decode_subfrm5(buff,alm,ion,utc,leaps);
  }
	return ID_ERROR;
}	
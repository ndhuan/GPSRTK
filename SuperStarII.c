#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rtk.h"


#define SOH_BYTE 0x01
#define ID_SS2LLH   20          /* ss2 message ID#20 navigation data (user) */
#define ID_SS2ECEF  21          /* ss2 message ID#21 navigation data (ecef) */
#define ID_SS2EPH   22          /* ss2 message ID#22 ephemeris data */
#define ID_SS2RAW   23          /* ss2 message ID#23 measurement block */
#define ID_SS2SBAS  67          /* ss2 message ID#67 sbas data */

static uint16_t U2(uint8_t* p)
{
//	uint16_t temp;
//	memcpy(&temp,p,2);
//	return temp;
	return *((uint16_t*)p);
}
static uint32_t U4(uint8_t* p)
{
//	uint32_t temp;
//	memcpy(&temp,p,4);
//	return temp;
	return *((uint32_t*)p);
}
static double R8(uint8_t* p)
{
	double temp;
	memcpy(&temp,p,8);
	return temp;
}
void adjweek(raw_t* raw,double tow)
{
	int week;
	double sec;
	if (raw->time.time == 0)
			return;
	sec = time2gpst(raw->time,&week);
	if (tow-sec<-SEC_PER_HALF_WEEK)
		tow += SEC_PER_WEEK;
	else if (tow-sec>SEC_PER_HALF_WEEK)
		tow -= SEC_PER_WEEK;
	raw->time = gpst2time(week,tow);
}
Error decode_ss2llh(raw_t* raw)
{
	double ep[6];
	uint8_t* p = raw->buff;
	if (raw->len != 77)
		return LEN_ERROR;
	ep[0]=U2(p+16);
	ep[1]=p[15];
	ep[2]=p[14];
	ep[3]=(p[4]&0x1f);
	ep[4]=p[5];
	ep[5]=R8(p+6);
	raw->time = utc2gpst(epoch2time(ep));
	return NO_ERROR1;
//	return ID_20;
}
Error decode_ss2ecef(raw_t* raw)
{
	uint8_t* p = raw->buff;
	if (raw->len != 85)
		return LEN_ERROR;
	raw->time = gpst2time(U2(p+12),R8(p+4));
	return NO_ERROR1;
//	return ID_21;
}
Error decode_ss2eph(raw_t* raw)
{
	Error error;
	uint8_t buff[90]={0};
	eph_t eph = {0};
	uint8_t* p = raw->buff;
	
	uint8_t sat;
	int i,j;
	if (raw->len != 79)
		return LEN_ERROR;
	
	if (raw->time.time == 0)//time tu receiver do neu
		//dung gps tow thi bi week rollover
		return TIMING_ERROR;
	
//	tow = (uint32_t)((time2gpst(raw->time,NULL))/6);
	for (i=0;i<3;i++)
	{
//		buff[30*i+3]=(uint8_t)(tow>>9);
//		buff[30*i+4]=(uint8_t)(tow>>1);
//		buff[30*i+5]=(uint8_t)(((tow&1)<<7)+((i+1)<<2));
		buff[30*i+5]=(uint8_t)((i+1)<<2);
		for (j=0;j<24;j++)
		{
			buff[30*i+6+j]=*(p+5+24*i+j);
		}
	}
	//time tu receiver do neu
	//dung gps tow thi bi week rollover
	//eph.ttr = raw->time;
	time2gpst(raw->time,&eph.week);
	
	error = decode_subfrm1(buff,&eph);
	if (error != NO_ERROR2)
		return error;
	error = decode_subfrm2(buff+30,&eph);
	if (error != NO_ERROR2)
		return error;
	error = decode_subfrm3(buff+60,&eph);
	if (error != NO_ERROR2)
		return error;	
	sat = (p[4]&0x1f);
	if (eph.iode == raw->nav.eph[sat].iode)
		return UNCHANGE;
	eph.sat = sat+1;
	eph.ttr=raw->time;
	raw->nav.eph[sat]=eph;
	raw->ephsat = sat+1;
//	return NO_ERROR;
	return EPHEMERIS;
}
Error decode_ss2meas(raw_t* raw)
{
	const double FREQIF = 1.405396825e3, TIMESLEW=1.75e-7;
	uint8_t* p = raw->buff;
	double tow,slew,P,icp;
	int i,n;
	unsigned int sc;
	int nobs = p[6];
	if ((17+11*nobs) != raw->len)
		return LEN_ERROR;
	tow = (int)((R8(p+7)*1000+0.5))/1000;
	adjweek(raw, tow);
	slew = (int8_t)p[4]*TIMESLEW;
	raw->icpc += 4.5803 - slew*FREQIF - FREQ1*(slew-1e-6);//phase correction P144
	for (i=0;i<nobs;i++,p+=11)
	{
		n = (p[15]&0x1f);
		raw->obs.data[i].time = raw->time;
		raw->obs.data[i].sat = n+1;
		P=(tow-(int)tow)-(double)U4(p+17)/2095104000;//P142
		raw->obs.data[i].P = (P+((P<0)?1:0))*CLIGHT;
		icp = (double)(U4(p+21)>>2)/1024.0+raw->off[n];//unwrap P140
		if (icp-(raw->icpp[n])>524288.0)
		{
			icp -= 1048576.0;
			raw->off[n] -= 1048576.0;
		}
		else if (icp-(raw->icpp[n])<-524288.0)
		{
			icp += 1048576.0;
			raw->off[n] += 1048576.0;
		}
		raw->icpp[n] = icp;
		raw->obs.data[i].L = icp + raw->icpc;
		raw->obs.data[i].D = 0.0;
		raw->obs.data[i].SNR = (unsigned char)(floor(p[16])+0.5);
		sc = p[25];
		raw->obs.data[i].LLI = (int)((uint8_t)sc-(uint8_t)raw->lockt[n][0])>0;//?????????????///
		raw->obs.data[i].LLI |= p[21]&1?2:0;//???????
		raw->lockt[n][0] = sc;
	}
	raw->obs.n = i;
//	return NO_ERROR;
	return OBS;
}
bool sync_ss2(uint8_t* buff, uint8_t data)
{
	buff[0] = buff[1];
	buff[1] = buff[2];
	buff[2] = data;
	return ((buff[0] == SOH_BYTE) && (buff[1]+buff[2]==0xff));
}
bool check_sum(uint8_t* buff,uint8_t len)
{
	int i=0;
	uint16_t sum=0;	
	for (i=0;i<len-2;i++)
	{
		sum += buff[i];
	}
	return (((sum>>8)==buff[len-1]) && ((sum&0xff)==buff[len-2]));
}
/* -- bool decode_ss2(raw_t* raw, uint8_t data) --------------------------------------
 * 
 * Description	:  
 * Parameters	: 
 * Return		: type of error
 */
Error decode_ss2(raw_t* raw)
{
	uint8_t type;
	if (!check_sum(raw->buff, raw->len))
	{
		return CHECKSUM_ERROR;
	}
	type = raw->buff[1];
	switch (type)
	{
		case ID_SS2LLH:
		{
//			return NO_ERROR;
			return decode_ss2llh(raw);
		}
		case ID_SS2ECEF:
		{
//			return NO_ERROR;
			return decode_ss2ecef(raw);
		}
		case ID_SS2EPH:
		{
//			return NO_ERROR;
			return decode_ss2eph(raw);
		}
		case ID_SS2RAW:
		{
//			return NO_ERROR;
			return decode_ss2meas(raw);
		}
	}
	return NO_ERROR1;
}
/* -- bool input_ss2(raw_t* raw, uint8_t data) --------------------------------------
 * 
 * Description	:  
 * Parameters	: 
 * Return		: 
 */
Error input_ss2(raw_t* raw, uint8_t data)
{

	
		if ((raw->nbyte) == 0)
		{
			if (!sync_ss2(raw->buff,data))
				return INCOMPLETE;
			raw->nbyte = 3;
			return INCOMPLETE;		
		}
		raw->buff[raw->nbyte++] = data;
		if (raw->nbyte < 4)
			return INCOMPLETE;
		if (raw->nbyte == 4)
		{			
			raw->len = data+6;
		}
		if (raw->nbyte == raw->len)
		{
			raw->nbyte = 0;
			return decode_ss2(raw);
		}
		return INCOMPLETE;
}
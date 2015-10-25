#include <cstdlib>
#include "main.h"
#include "rtk.h"

static eph_t eph[2*MAX_SAT] __attribute__((section("IRAM2")));
static eph_t REph[MAX_SAT];
static eph_t BEph[MAX_SAT];
//static alm_t Ralm[MAX_SAT];
//static alm_t Balm[MAX_SAT];
//static obsd_t RObsData[MAX_OBS] ;
//static obsd_t BObsData[MAX_OBS];

static char result[SOL_MSG_LEN];// __attribute__((section("IRAM1")));
static uint8_t roverBuff[MAX_RAW_LEN];// __attribute__((section("IRAM1")));
static uint8_t baseBuff[MAX_RAW_LEN];// __attribute__((section("IRAM1")));

/* -- void rtksvrstart(rtksvr_t* svr) --------------------------------------
 * 
 * Description	: 
 * Parameters	: 
 * Return		: 
 */
void rtksvrstart(rtksvr_t* svr)
{
	int i;
//	sol_t sol0 = {{0}};//time
	eph_t eph0 = {0,-1,-1};//sat numer, iode, iodc
	gtime_t time0={0};

	rtkinit(&svr->rtk,&default_opt);//base position is set here
	
	svr->buff[0] = roverBuff;
	svr->buff[1] = baseBuff;
	svr->nb[0]=svr->nb[1]=0;
	svr->buffPtr[0]=svr->buffPtr[1]=0;
	
	svr->rtk.sol.result = result;
		
	init_raw(svr->raw,REph);
	init_raw(svr->raw+1,BEph);	
	
	svr->ftime = time0;	
	
	svr->obs[0].n=0;
	svr->obs[1].n=0;
	
//	svr->nav.eph =(eph_t  *)malloc(sizeof(eph_t )*MAX_SAT *2);
		svr->nav.eph = eph;
	
	for (i=0;i<2*MAX_SAT;i++)
	{
		svr->nav.eph[i]=eph0;		
		svr->nav.eph[i].ttr = time0;
		svr->nav.eph[i].sat=0;
	}
	
	svr->nav.n = 2*MAX_SAT;
	
	svr->format[0] = STRFMT_UBX;//rover
	svr->format[1] = STRFMT_UBX;//base
	//svr->format[0] = STRFMT_SS2;
	//svr->format[1] = STRFMT_SS2;
}
/* -- void updatesvr(rtksvr_t* svr,Error Err, int index) --------------------------------------
 * 
 * Description	: 
 * Parameters	: index 
 *								0 rover
 *								1 base	
 * Return		: 
 */
void updatesvr(rtksvr_t* svr,Error Err, int index)
{
	int i,sat;
	obs_t *obs,*svrobs;
	eph_t *eph1,*eph2,*eph3;
	
	switch(Err)
	{
		case (OBS):
		{
			obs =&svr->raw[index].obs;
			svrobs = svr->obs+index;
			svrobs->n=0;
			for (i=0;(i<obs->n)&&(i<(MAX_OBS>>1));i++)
			{
				svrobs->data[svrobs->n]=obs->data[i];
				svrobs->data[(svrobs->n)++].rcv=index+1;
			}			
			break;
		}
		case (EPHEMERIS):
		{			
			sat = svr->raw[index].ephsat;
			eph1 = svr->raw[index].nav.eph+sat-1;
			eph2 = svr->nav.eph+sat-1;
			eph3 = eph2 + MAX_SAT;
			if ((eph2->ttr.time == 0) ||
				((eph1->iode!=eph2->iode)&&(eph1->iode!=eph3->iode)))
			{
				*eph3 = *eph2;
				*eph2 = *eph1;
			}
			break;
		}
	}
}
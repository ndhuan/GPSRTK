#include <math.h>
#include "rtk.h"
#include "main.h"

#define STD_BRDCCLK 30.0          /* error of broadcast clock (m) */
/* variance by ura ephemeris (ref [1] 20.3.3.3.1.1) --------------------------*/
static double var_uraeph(int ura)
{
    const double ura_value[]={   
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0
    };
    return ura<0||15<ura?SQR(6144.0):SQR(ura_value[ura]);
}
//satellite clock does not include relativity correction and tdg
/* -- double eph2clk(gtime_t time,const eph_t *eph) --------------------------------------
 * 
 * Description	: broadcast ephemeris to sat clock bias
 * Parameters	: time	I	time by sat clock
 *							eph		I	broadcast ephemeris
 * Return		: sat clock bias
 */
double eph2clk(gtime_t time,const eph_t *eph)
{
	double t;
	int i;
	t=timediff(time,eph->toc);
	for (i=0;i<2;i++)
	{
		t -= eph->f0 + eph->f1*t + eph->f2*t*t;
	}
	return eph->f0 + eph->f1*t + eph->f2*t*t;
}
/* -- void eph2pos(gtime_t time, const eph_t *eph,double *rs,double *dts,double *var)--------------------------------------
 * Description	: broadcast ephemeris to sat position and clock bias 
 * Parameters	: 
 *							time 	I		transmission time Tsv
 *							rs		O		sat positions and velocities
 *							dts		O		sat clock correction (bias, drift)
 *							var		O		sat positions and clock error variance
 * Return		: 0	no ephemeris
 */
void eph2pos(gtime_t time, const eph_t *eph,double *rs,double *dts,double *var)
{
	double Mk,tk,Ek,E,sinEk,cosEk,phik,sin2p,cos2p;
	double uk,rk,ik,e=eph->e,xk,yk;
	tk=timediff(time,eph->toe);
	Mk=eph->M0+(sqrt(MUY/(eph->A*eph->A*eph->A))+eph->deln)*tk;
			
			
	for (Ek=Mk,E=0.0;fabs(Ek-E)>1e-14;)//newton method
	{
		E=Ek;
		Ek-=(Ek-e*sin(Ek)-Mk)/(1.0-e*cos(Ek));
	}


	//start=TIM2->CNT;	
	sinEk=sin(Ek);
	//SendIntStr(TIM2->CNT-start);
	cosEk=cos(Ek);
	phik=atan2(sqrt(1-e*e)*sinEk,cosEk-e)+eph->omg;
	sin2p=sin(2.0*phik);
	cos2p=cos(2.0*phik);
	uk=phik+eph->cus*sin2p+eph->cuc*cos2p;
	rk=eph->A*(1-e*cosEk)+eph->crs*sin2p+eph->crc*cos2p;
	ik=eph->i0+eph->cis*sin2p+eph->cic*cos2p+eph->idot*tk;
	xk=rk*cos(uk);
	yk=rk*sin(uk);
	phik=eph->OMG0+(eph->OMgd-OMGE)*tk-OMGE*eph->toes;
	sin2p=sin(phik);
	cos2p=cos(phik);
	cosEk=cos(ik);
	rs[0]=xk*cos2p-yk*cosEk*sin2p;
	rs[1]=xk*sin2p+yk*cosEk*cos2p;
	rs[2]=yk*sin(ik);
	tk=timediff(time,eph->toc);
	(*dts)=(eph->f0)+(eph->f1)*tk+(eph->f2)*tk*tk;
	(*dts)+=F*e*sqrt(eph->A)*sinEk;//hieu ung tuong doi tinh
	
	*var=var_uraeph(eph->sva);
}


eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav, char **msg)
{
	double t,tmin,tmax;
	int i,j=-1;
	tmax=MAXDTOE+1.0;
	tmin=tmax+1.0;
	

	for (i=sat-1;i<2*MAX_SAT;i+=MAX_SAT)
	{
		if (nav->eph[i].sat!=sat) continue;
		//*msg += sprintf(*msg,"seleph1");
		if (iode>=0&&nav->eph[i].iode!=iode)continue;
		//*msg += sprintf(*msg,"seleph2");
		if ((t=fabs(timediff(nav->eph[i].toe,time)))>tmax) continue;
		//*msg += sprintf(*msg,"seleph3");
		if (iode>=0) return nav->eph+i;
		//*msg += sprintf(*msg,"seleph4");
		if (t<=tmin)/* choose eph that has toe closest to time */
		{
			j=i;
			tmin=t;
		}
	}
	if (iode>=0||j<0)
	{
		//*msg += sprintf(*msg,"seleph5");
		return NULL;
	}
	return nav->eph+j;
}
/* satellite clock with broadcast ephemeris ----------------------------------*/
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  double *dts,char **msg)
{
    eph_t  *eph;
		if (!(eph=seleph(teph,sat,-1,nav,msg))) return 0;
    //*msg += sprintf(*msg,"ephclk");
		*dts=eph2clk(time,eph);
    return 1;
}
/* satellite position and clock by broadcast ephemeris -----------------------*/
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh,char **msg)
{
    eph_t  *eph;
    double rst[3],dtst[1],tt=1E-3;
    int i;
    *svh=-1;
    if (!(eph=seleph(teph,sat,iode,nav,msg))) return 0;
    //start=TIM2->CNT;
		eph2pos(time,eph,rs,dts,var);
		//SendIntStr(TIM2->CNT-start);
    time=timeadd(time,tt);
    eph2pos(time,eph,rst,dtst,var);
    *svh=eph->svh;
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
    dts[1]=(dtst[0]-dts[0])/tt;
    
    return 1;
}

/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst)
*          int    sat       I   satellite number
*          nav_t  *nav      I   navigation data
*          int    ephopt    I   ephemeris option (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)
*                               {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts      O   sat clock {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
extern int satpos(gtime_t time, gtime_t teph, int sat,
                  const nav_t *nav, double *rs, double *dts, double *var,
                  int *svh,char **msg)
{
    *svh=0;
    return ephpos(time,teph,sat,nav,-1,rs,dts,var,svh,msg);
//		return 1;
}
/* -- void satposs(gtime_t teph,const obsd_t *obs,int n,const nav_t *nav,
		double *rs, double *dts, double *var, int *svh) --------------------------------------
 * 
 * Description	: 
 * Parameters	: *rs		O	satellite positions and velocities (ecef), size n
 *							*dts	O	sat clocks
 *							*var	O	sat position and clock error variances (m^2)
 *							*svh	O sat health flag
 * Return		: 
 */
void satposs(gtime_t teph,const obsd_t *obs,int n,const nav_t *nav,
		double *rs, double *dts, double *var, int *svh,char **msg)
{
	gtime_t time[MAX_OBS]={{0}};
	double dt;
	int i,j;
	
	for (i=0;(i<n)&&(i<MAX_OBS);i++)
	{
		for (j=0;j<6;j++) rs[j+i*6]=0.0;
		for (j=0;j<2;j++) dts[j+i*2]=0.0;
		var[i]=0.0;svh[i]=0.0;
		if (obs[i].P==0.0)
		{			
			continue;
		}
		
		/* transmission time by satellite clock */
		time[i]=timeadd(obs[i].time,-obs[i].P/CLIGHT);
		/* satellite clock bias by broadcast ephemeris */
    if (!ephclk(time[i],teph,obs[i].sat,nav,&dt,msg)) {
      continue;
    }
		//*msg += sprintf(*msg,"clk");
		time[i]=timeadd(time[i],-dt);
		//start = TIM2->CNT;
		/* satellite position and clock at transmission time */
		if (!satpos(time[i],teph,obs[i].sat,nav,rs+i*6,dts+i*2,var+i,
                    svh+i,msg)) {

				continue;
    }
		//if (i==0)
		//		SendIntStr(TIM2->CNT-start);
		//*msg += sprintf(*msg,"satpos");								
		//if no precise clock available, use broadcast clock instead
		if (dts[i*2]==0.0)
		{
      if (!ephclk(time[i],teph,obs[i].sat,nav,dts+i*2,msg)) 
				continue;
      dts[1+i*2]=0.0;
			*var=SQR(STD_BRDCCLK);
		}
	}
}			
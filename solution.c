#include <stdio.h>
#include <math.h>
#include "main.h"
#include "rtk.h"

#define SQRT(x)    ((x)<0.0?0.0:sqrt(x))



static double sqvar(double covar)
{
    return covar<0.0?-sqrt(-covar):sqrt(covar);
}
extern void outsol(char* res, const sol_t *sol, const double *rb)
{
	gtime_t time; 
	double ep[6],pos[3],Qecef[9],Qenu[9];
#ifdef SOLF_ENU
	double e[3],enu[3],rr[3],eQ[9];
	int i=0;
#endif	
	
	time=sol->time;
	time2epoch(time,ep);
	
#ifdef SOLF_LLH	
	ecef2pos(sol->rr,pos);
	Qecef[0]=sol->qr[0];//xx
	Qecef[4]=sol->qr[1];//yy
	Qecef[8]=sol->qr[2];//zz
	Qecef[1]=Qecef[3]=sol->qr[3];//xy
	Qecef[5]=Qecef[7]=sol->qr[4];//yz
	Qecef[2]=Qecef[6]=sol->qr[5];//zx
	covenu(pos,Qecef,Qenu);
	
	res+=sprintf(res,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%06.3f %14.9f %14.9f %10.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f",
	ep[0],ep[1],ep[2],ep[3],ep[4],ep[5],//time yy/mm/dd hh:mm:ss.ssss
	pos[0]*R2D,pos[1]*R2D,pos[2],
	sol->stat,sol->ns,
	SQRT(Qenu[4]),SQRT(Qenu[0]),SQRT(Qenu[8]),
	sqvar(Qenu[1]),sqvar(Qenu[2]),sqvar(Qenu[5]),
	sol->age,sol->ratio);
#elif defined SOLF_ENU
	ecef2pos(rb,pos);
	Qecef[0]=sol->qr[0];//xx
	Qecef[4]=sol->qr[1];//yy
	Qecef[8]=sol->qr[2];//zz
	Qecef[1]=Qecef[3]=sol->qr[3];//xy
	Qecef[5]=Qecef[7]=sol->qr[4];//yz
	Qecef[2]=Qecef[6]=sol->qr[5];//zx
	xyz2enu(pos,e);
	for (i=0;i<3;i++)
		rr[i]=sol->rr[i]-rb[i];
	matmul("NN",3,1,3,1.0,e,rr,0.0,enu);
	matmul("NN",3,3,3,1.0,e,Qecef,0.0,eQ);
  matmul("NT",3,3,3,1.0,eQ,e,0.0,Qenu);	
	
	res+=sprintf(res,"%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%06.3f %14.4f %14.4f %14.4f %3d %3d %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f",
	ep[0],ep[1],ep[2],ep[3],ep[4],ep[5],//time yy/mm/dd hh:mm:ss.ssss
	enu[0],enu[1],enu[2],
	sol->stat,sol->ns,
	SQRT(Qenu[4]),SQRT(Qenu[0]),SQRT(Qenu[8]),
	sqvar(Qenu[1]),sqvar(Qenu[2]),sqvar(Qenu[5]),
	sol->age,sol->ratio);
#endif

#ifdef ENCODER
	res+=sprintf(res," %7d",sol->encoder);
#endif

#ifdef TIME_MEASURE
	res+=sprintf(res," %4d",sol->processTime);
#endif

	res[0]='\n';		
	
}
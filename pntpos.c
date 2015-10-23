#include <string.h>
#include <math.h>
#include <cstdlib>
#include <stdio.h>
#include "rtk.h"
#include "main.h"

#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay std (m) */
#define ERR_TROP    3.0         /* tropspheric delay std (m) */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5         /* broadcast iono model error factor */
#define ERR_CBIAS   0.3         /* code bias error std (m) */
#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */
//#define R						100.0
//#define a						0.03
//#define b						0.03

//const double chi_square[]={10.83,13.82,16.27,18.47,20.51};//0.1%
/* pseudorange measurement error variance ------------------------------------*/
static double varerr(const prcopt_t *opt, double el)
{
    double fact,varr;
    fact=EFACT_GPS;
    varr=SQR(opt->err[0])*(SQR(opt->err[1])+SQR(opt->err[2])/sin(el));
    return SQR(fact)*varr;
}
//0:ok,!0:not
extern int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
                    const double *azel, double *ion, double *var)
{
    /* broadcast model */
    
        *ion=ionmodel(time,nav->ion_gps,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 0;
    
    /* sbas ionosphere model */
//    if (ionoopt==ION_SBAS) {
//        return sbsioncorr(time,nav,pos,azel,ion,var);
//    }
//		return -1;
}
//0:ok,!0:not
extern int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, double *trp, double *var)
{
	*trp=tropmodel(time,pos,azel,REL_HUMI);
  *var=SQR(ERR_SAAS/(sin(azel[1])+0.1));
  return 0;
}

/*	
Compute residual and other components for LSE
args:	int *svh				I		SV health (-1: ephemeris unavailable,!=0 unhealthy sat)
			int n						I		number of observation data
			double *rs			I		sat pos/vel (ecef), size n
			double *dts			I		sat clock bias
			double *x				I		receiver pos/clock bias	
			double *azel 		O		{az,el} (rad), size 2n
			double *H				O		design matrix
			double *v				O		residual of all obs data
			double *resp		O   residual of valid obs data
			double *var			O		residual error variance
			double *nv			O		number of valid sat
			int *vsat				O		valid sat flag
*/
static void rescode(const obsd_t *obs, const nav_t *nav,const int *svh,
		int n, double *rs, double *dts,double *azel, double *H, 
		double *v,double *var_sat,double *var, double *resp,double *x,int *nv,int *vsat,const prcopt_t *opt, char **msg)
{
	int i,sat;
	double pos[3],e[3];
	double P,ion,trop,var_P,var_ion,var_trop,r;
	*nv=0;
	ecef2pos(x,pos);
	for (i=0;i<n;i++)
	{
		vsat[i]=0;
		azel[i*2]=azel[i*2+1]=resp[i]=0.0;
		if (svh[i])//unhealthy satellite
		{			
			//**msg='s';
			//(*msg)++;
			continue;			
		}
		if ((r=geodist(rs+i*6,x,e))<=0) 
		{						
			//**msg='r';
			//(*msg)++;
			continue;
		}
		if (satazel(pos,e,azel+i*2)<opt->elmin)
		{			
			//**msg='a';
			//(*msg)++;
			continue;
		}
		//pseudorange (group delay correction)
		sat=(obs+i)->sat;
		P=(obs+i)->P;
		if (nav->eph[sat-1].sat==sat)
			P-=nav->eph[sat-1].tgd[0]*CLIGHT;
		else if (nav->eph[MAX_SAT+sat-1].sat==sat)
			P-=nav->eph[MAX_SAT+sat-1].tgd[0]*CLIGHT;
		if (P==0.0)
			continue;
		var_P = SQR(ERR_CBIAS);
		//ionosheric corrections: check opt for sbas|broadcast model
		ionocorr(obs[i].time,nav,obs[i].sat,pos,azel+i*2,&ion,&var_ion);
		//tropospheric corrections: 
		tropcorr(obs[i].time,nav,pos,azel+2*i,&trop,&var_trop);
		
		//*msg+=sprintf(*msg,"%f %f %f %f %f %f\n",P,r,ion,trop,x[3],dts[2*i]);
		
		v[*nv]=P-(r+ion+trop+x[3]-CLIGHT*dts[i*2]);
		vsat[i]=1;resp[i]=v[*nv];

		H[(*nv)*4]=-e[0];
		H[(*nv)*4+1]=-e[1];
		H[(*nv)*4+2]=-e[2];		
		H[(*nv)*4+3]=1.0;
		var[*nv]=varerr(opt,azel[1+i*2])+var_sat[i]+var_ion+var_trop+var_P;
		(*nv)++;
	}
}
static int solval(double *v,int nv,char **msg,double *azel,int n, int *vsat,const prcopt_t *opt)
{
	int i,ns;
	double tmp,azels[2*MAX_OBS],dop[4];
	//chi square test
	if (nv>4)
	{
		tmp=dot(v,v,nv);
		if (tmp>=chisqr[nv-5])
		{
#ifdef _DEBUG_MSG			
			strcpy(*msg,"chi square test failed\n");
			(*msg)+=sizeof("chi square test failed\n");
#endif			
			return -1;
		}
	}
	for (i=0,ns=0;i<n;i++)
	{
		if (vsat[i])
		{
			azels[2*ns]=azel[2*i];
			azels[2*ns+1]=azel[2*i+1];
			ns++;
		}
	}
	//GDOP test: azels contains only valid sat {az,el}
	dops(ns,azels,opt->elmin,dop);
	if ((dop[0]<=0)||(dop[0]>opt->maxgdop))
	{
#ifdef _DEBUG_MSG
		(*msg) += sprintf(*msg,"GDOP error:%f,ns %d\n",dop[0],ns);
#endif		
		return -1;
	}
	return 0;
}



/* int estpos(const obsd_t *obs, const nav_t *nav,int n, double *rs, double *dts,
		double *resp,double *sva, sol_t *sol, double *azel, int *vsat, char *msg,
		prcopt_t *opt)
Description:
Args:	obsd_t *obs			I			obs data
			nav_t* nav			I			nav data
			int n						I			number of obs data
			int *svh				I			SV health (-1 if no correction available)
			double *rs			O			sat pos/velocities
			double *dts			O			sat clock bias/clock drift rate
			double *resp		O			pseudocode residual (size n)
			double *var			O			pseudorange residual error variance
			double *azel		O			{az/el} rad (size 2n)
			int *vsat				O			valid flags (size n)
			prcopt_t *opt		I			option
			sol_t *sol			O			solution
			char *msg						
Return: 0 if ok
*/
static int estpos(const obsd_t *obs, const nav_t *nav,int n, const int* svh,double *rs, double *dts,
	double *resp,double *var_sat, double *azel,int *vsat, const prcopt_t *opt,sol_t *sol,char **msg)
{
	int i,j,k,nv=0;
	double x[4],dx[4],Q[16],tmp;
	double *H, *v, *var;
//	double H[4*MAX_OBS],v[MAX_OBS],var[MAX_OBS];

//	double P,ion,trop,r,var_P,var_ion,var_trop;
	H=mat(4,n);v=mat(n,1);var=mat(n,1);
	if ((!H)||(!v)||(!var))
	{
#ifdef _DEBUG_MSG
		*msg+=sprintf(*msg,"estpos mat allocate error");
#endif		
		free(H);free(v);free(var);
		return -1;
	}
	for (i=0;i<3;i++) x[i]=sol->rr[i];
	x[3]=0.0;
//	msg+=sprintf(msg,"x=%f %f %f %f\n",x[0],x[1],x[2],x[3]);
	for (i=0;i<MAXITR;i++)
	{
		rescode(obs,nav,svh,n,rs,dts,azel,H,v,var_sat,var,resp,x,&nv,vsat,opt,msg);
//		if ((i==0)||(i==1))
//			msg+=sprintf(msg,"%f %f %f %f %f\n",P,r,ion,trop,x[3]);
//			msg+=sprintf(msg,"x:%f %f %f %f\n",x[0],x[1],x[2],x[3]);
			
//		if (i==0)
//		{
//			for (j=0;j<n;j++)
//			{
//				if (obs[j].sat==29)
//					msg+=sprintf(msg,"%f %f %f\n",rs[6*j],rs[6*j+1],rs[6*j+2]);
//			}
//		}
		if (nv<4)
		{
#ifdef _DEBUG_MSG
			(*msg)+=sprintf(*msg,"lack of valid sat:%d\n",nv);
#endif			
			free(H);free(v);free(var);
//			for (j=0;j<nv;j++)	
//				msg+=sprintf(msg,"%f %f %f %f\n",H[j*4],H[j*4+1],H[j*4+2],v[j]);			
//			sprintf(msg,"%d:%6.1f %d:%6.1f %d:%6.1f %d:%6.1f %d:%6.1f\n",
//				obs[0].sat,azel[1]*R2D,obs[1].sat,azel[3]*R2D,obs[2].sat,azel[5]*R2D,obs[3].sat,azel[7]*R2D,obs[4].sat,azel[9]*R2D);
			return 1;
		}
		//weight variance
		for (j=0;j<nv;j++)
		{
			tmp=sqrt(var[j]); 
			v[j]/=tmp;
			for (k=0;k<4;k++)
			{
				H[k+j*4]/=tmp;
			}
		}
//		if ((i==0)||(i==1))
//			msg+=sprintf(msg,"%f %f %f %f\n",H[0],H[1],H[2],v[0]);
//		for (j=0;j<nv;j++)	
//		if (i==1)
//			msg+=sprintf(msg,"H:%f %f %f v:%f\n",H[0],H[1],H[2],v[0]);			
		//start = TIM2->CNT;
		if (lsq(H,v,4,nv,dx,Q))
		{
#ifdef _DEBUG_MSG
			*msg += sprintf(*msg,"lsq error\n");
#endif			
			free(H);free(v);free(var);
//			(*msg) += sizeof("lsq error\n");
			return -1;
		}
		//if(i==0)
		//	SendIntStr(TIM2->CNT-start);
		
		for (j=0;j<4;j++)
		{
			x[j]+=dx[j];
		}
//		if ((i==0)||(i==1))
//			msg+=sprintf(msg,"x:%f %f %f %f\n",x[0],x[1],x[2],x[3]);
		if (sos4(dx)<1E-8){
			tmp=x[3]/CLIGHT;
			sol->time = timeadd(obs[0].time,-tmp);
			for (j=0;j<6;j++)
			{
				sol->rr[j]=(j<3)?x[j]:0.0;
			}
			sol->qr[0]=Q[0];//varxx
			sol->qr[1]=Q[5];//varyy
			sol->qr[2]=Q[10];//varzz
			sol->qr[3]=Q[1];//varxy
			sol->qr[4]=Q[6];//varyz
			sol->qr[5]=Q[2];//varzx
			
			sol->dtr[0]=tmp;
			
			sol->type = 0;//xyz-ecef

			sol->ns=nv;
			
			sol->age = 0.0;
			sol->ratio = 0.0;
			
//			msg+=sprintf(msg,"%d %d x=%f %f %f %f\n",nv,i,x[0],x[1],x[2],x[3]);
			free(H);free(v);free(var);
			return solval(v,nv,msg,azel,n,vsat,opt);		
		}
	}
	if (i>=MAXITR)
	{
#ifdef _DEBUG_MSG
		*msg+=sprintf(*msg,"estpos diverged\n");
#endif		
//		(*msg) += sizeof("estpos diverged\n");
		
	}
	free(H);free(v);free(var);
	return -1;
}

extern int pntpos(const obsd_t *obs, int n, const nav_t *nav, sol_t *sol,
	double *azel, ssat_t *ssat, const prcopt_t *opt, char **msg)
{
	double *rs, *dts, *var_sat, *azel_, *resp;
	//	double rs[6*MAX_OBS],dts[2*MAX_OBS],var_sat[MAX_OBS],azel_[2*MAX_OBS],resp[MAX_OBS]; 
	int svh[MAX_OBS];//svh=-1: no correction available
	int i;
	int vsat[MAX_OBS]={0};//valid flag 
	
	if (n<=0)
	{
#ifdef _DEBUG_MSG		
		*msg += sprintf(*msg,"no obs data\n");
#endif		
		//(*msg) += sizeof("no obs data\n");
		return 1;
	}
	sol->time = obs[0].time;
	(*msg)[0]='\0';
//	for (i=0;i<2*MAX_OBS;i++)
//	{
//		azel_[i]=0.0;
//	}
	rs=mat(6,n);dts=mat(2,n);var_sat=mat(1,n);azel_=zeros(2,n);resp=mat(1,n);
	if ((!rs) || (!dts) || (!var_sat) || (!azel_) || (!resp))
	{
		*msg += sprintf(*msg,"pntpos mat allocate error\n");
		free(rs);free(dts);free(var_sat);free(azel_);free(resp);	
		return 1;
	}	
	//sat positions, vels, clocks (rs[i]={0.0,0.0,0.0} if ephemeris is unavailable)
	satposs(sol->time,obs,n,nav,rs,dts,var_sat,svh,msg);
		
	i = estpos(obs,nav,n,svh,rs,dts,resp,var_sat,azel_,vsat,opt,sol,msg);
	if (azel) {
		for (i=0;i<n*2;i++) azel[i]=azel_[i];
	}
	//SendIntStr(HAL_GetTick()-start);
	
	free(rs);free(dts);free(var_sat);free(azel_);free(resp);	
	return i;
}

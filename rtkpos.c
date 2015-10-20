#include "main.h"
#include "rtk.h"

#define SQRT(x)     ((x)<=0.0?0.0:sqrt(x))

#define VAR_HOLDAMB 0.001    /* constraint to hold ambiguity (cycle^2) */

#define VAR_POS     SQR(30.0) /* initial variance of receiver pos (m^2) */

#define NF(opt)     1
#define NP(opt)     ((opt)->dynamics==0?3:9)//3
#define NI(opt)     0 //(broadcasr model or klobuchar)
#define NT(opt)     0 // (Saastamoinen model)
#define NL(opt)     0 //0 glonass model AR off
#define NB(opt)     32//kinematic mode=>32*1=32
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))//3
#define NX(opt)     (NR(opt)+NB(opt))

/*
#define NF(opt)     1//((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)
#define NP(opt)     ((opt)->dynamics==0?3:9)
#define NI(opt)     ((opt)->ionoopt!=IONOOPT_EST?0:MAX_SAT)
#define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))
#define NL(opt)     ((opt)->glomodear!=2?0:NFREQGLO)
#define NB(opt)     ((opt)->mode<=PMODE_DGPS?0:MAX_SAT*NF(opt))
#define NR(opt)     (NP(opt)+NI(opt)+NT(opt)+NL(opt))
#define NX(opt)     (NR(opt)+NB(opt))
*/
#define IB(s,f,opt) (NR(opt)+MAX_SAT*(f)+(s)-1) /* phase bias (s:satno,f:freq) */
#define NITER 1
/* single-differenced observable ---------------------------------------------*/
static double sdobs(const obsd_t *obs, int i, int j, int f)
{
    double pi=(f<1)?obs[i].L:obs[i].P;
    double pj=(f<1)?obs[j].L:obs[j].P;
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
/* single-differenced measurement error variance -----------------------------*/
static double varerr(int sat, int sys, double el, double bl, double dt, int f,
                     const prcopt_t *opt)
{
    double a,b,c=opt->err[3]*bl/1E4,d=CLIGHT*opt->sclkstab*dt,fact=1.0;
    double sinel=sin(el);
    int nf=1;
		/* normal error model */
    if (f>=nf) fact=opt->eratio;
    if (fact<=0.0)  fact=opt->eratio;
		fact*=EFACT_GPS;
    a=fact*opt->err[1];
    b=fact*opt->err[2];
    
    return 2.0*(a*a+b*b/sinel/sinel+c*c)+d*d;
}
static double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i=0;i<3;i++) dr[i]=ru[i]-rb[i];
    return norm(dr,3);
}
/* initialize state and covariance -------------------------------------------*/
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;
    for (j=0;j<rtk->nx;j++) {
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=((i==j)?var:0.0);
    }
}
/* select common satellites between rover and reference station --------------*/
static int selsat(const obsd_t *obs, double *azel, int nu, int nr,
                  const prcopt_t *opt, int *sat, int *iu, int *ir,char **msg)
{
    int i,j,k=0,tmp;   
    for (i=0;i<nu;i++) 
		{
			tmp = (obs[i].sat);
			for (j=nu;j<nu+nr;j++)
			{	
				if (tmp == (obs[j].sat)) 
				{ /* elevation at base station */
//					(*msg)+=sprintf(*msg,"%d:%f ",tmp,azel[1+j*2]);
					if (azel[1+j*2]>=opt->elmin)
					{
						sat[k]=tmp; iu[k]=i; ir[k++]=j;
					}
					break;
				}					
      }
    }
    return k;
}
/* temporal update of position/velocity/acceleration -------------------------*/
static void udpos(rtk_t *rtk, double tt)
{
 //   double *F,*FP,*xp,pos[3],Q[9]={0},Qv[9],var=0.0;
    int i;
    /* initialize position for first epoch */
    if (sos3(rtk->x)<=0.0) {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
    }
    /* kinmatic mode without dynamics */
    for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
}

/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv,char **msg)
{
    unsigned char slip,LLI1,LLI2,LLI;
    int sat=obs[i].sat;
       
		if (obs[i].L==0.0) return;
		
		/* restore previous LLI */
		LLI1=(rtk->ssat[sat-1].slip>>6)&3;
		LLI2=(rtk->ssat[sat-1].slip>>4)&3;
		LLI=rcv==1?LLI1:LLI2;
		
		/* detect slip by cycle slip flag */
		slip=(rtk->ssat[sat-1].slip|obs[i].LLI)&3;
		
		if (obs[i].LLI&1) {
//				(*msg)+=sprintf(*msg,"slip detected (sat=%2d rcv=%d LLI %x)\n",
//							 sat,rcv,obs[i].LLI);
		}
		/* detect slip by parity unknown flag transition */
		if (((LLI&2)&&!(obs[i].LLI&2))||(!(LLI&2)&&(obs[i].LLI&2))) {
//				(*msg)+=sprintf(*msg,"slip detected (sat=%2d rcv=%d LLI %x->%x)\n",
//							 sat,rcv,LLI,obs[i].LLI);
				slip|=1;
		}
		/* save current LLI and slip flag */
		if (rcv==1) rtk->ssat[sat-1].slip=(obs[i].LLI<<6)|(LLI2<<4)|slip;
		else        rtk->ssat[sat-1].slip=(obs[i].LLI<<4)|(LLI1<<6)|slip;
    
}
static void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                   const int *iu, const int *ir, int ns, const nav_t *nav,char **msg)
{
	double cp,pr,*bias,offset;
	int i,j,slip,reset;
			
	for (i=0;i<ns;i++) 
	{			
		/* detect cycle slip by LLI */
		rtk->ssat[sat[i]-1].slip&=0xFC;
		detslp_ll(rtk,obs,iu[i],1,msg);
		detslp_ll(rtk,obs,ir[i],2,msg);
	}

	/* reset phase-bias if expire obs outage counter */
	for (i=1;i<=MAX_SAT;i++) {
			
			reset=++rtk->ssat[i-1].outc>(unsigned int)rtk->opt.maxout;
			if (reset&&rtk->x[IB(i,0,&rtk->opt)]!=0.0) {
					initx(rtk,0.0,0.0,IB(i,0,&rtk->opt));
			}
			if (reset) {
					rtk->ssat[i-1].lock=-rtk->opt.minlock;
			}
	}
	/* reset phase-bias if instantaneous AR or expire obs outage counter */
  for (i=1;i<=MAX_SAT;i++) {
            
		reset=++rtk->ssat[i-1].outc>(unsigned int)rtk->opt.maxout;
							
		if (reset&&rtk->x[IB(i,0,&rtk->opt)]!=0.0) {
			initx(rtk,0.0,0.0,IB(i,0,&rtk->opt));
		}
		if (rtk->opt.modear!=ARMODE_INST&&reset) {
			rtk->ssat[i-1].lock=-rtk->opt.minlock;
		}
  }
	/* reset phase-bias if detecting cycle slip */
	for (i=0;i<ns;i++) {
			j=IB(sat[i],0,&rtk->opt);
			rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*tt;
			slip=rtk->ssat[sat[i]-1].slip;
			if (!(slip&1)) continue;
			rtk->x[j]=0.0;
			rtk->ssat[sat[i]-1].lock=-rtk->opt.minlock;
	}
	bias=zeros(ns,1);
        
	/* estimate approximate phase-bias by phase - code */
	for (i=j=0,offset=0.0;i<ns;i++) 
	{
			cp=sdobs(obs,iu[i],ir[i],0); /* cycle */
			pr=sdobs(obs,iu[i],ir[i],1);
			
			if (cp==0.0||pr==0.0) continue;
			
			bias[i]=cp-pr*FREQ1/CLIGHT;
	
			if (rtk->x[IB(sat[i],0,&rtk->opt)]!=0.0) {
					offset+=bias[i]-rtk->x[IB(sat[i],0,&rtk->opt)];
					j++;
			}
	}
	/* correct phase-bias offset to enssure phase-code coherency */
	if (j>0) 
	{
		for (i=1;i<=MAX_SAT;i++) 
		{
			if (rtk->x[IB(i,0,&rtk->opt)]!=0.0) 
				rtk->x[IB(i,0,&rtk->opt)]+=offset/j;
		}
	}
	/* set initial states of phase-bias */
	for (i=0;i<ns;i++) 
	{
			if (bias[i]==0.0||rtk->x[IB(sat[i],0,&rtk->opt)]!=0.0) continue;
			initx(rtk,bias[i],SQR(rtk->opt.std[0]),IB(sat[i],0,&rtk->opt));
	}
	free(bias);
    
}
static void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
                    const int *iu, const int *ir, int ns, const nav_t *nav,char **msg)
{
    double tt=fabs(rtk->tt);//time difference between current and previous
//		double	bl,dr[3];
    
    /* temporal update of position/velocity/acceleration */
    udpos(rtk,tt);
    /* temporal update of phase-bias */
		udbias(rtk,tt,obs,sat,iu,ir,ns,nav,msg);
}
static void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
                      const double *azel,
                      const prcopt_t *opt, double *y)
{
  /* residuals = observable - pseudorange */
  if (testsnr(base,azel[1],obs->SNR*0.25,&opt->snrmask)) {
      return;
  }
	if (obs->L!=0.0) y[0]=obs->L*CLIGHT/FREQ1-r;
  if (obs->P!=0.0) y[1]=obs->P-r;
}
/*
static int zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const int *svh, const nav_t *nav,
                 const double *rr, const prcopt_t *opt, double *y,
                 double *e, double *azel)
Description: zero differenced residual
Params: 	base		I			1:base, 0: rover
					obs			I
					n				I			number of obs data
					rs			I			sat positions/velocities
					dts			I			sat clock bias
					svh			I			
					nav			I			
					rr			I			receiver position
					y				O			zero diff residuals
					e				O			
					azel		O			{az/el}
Return:	0 ok, !0 error
*/
static int zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const int *svh, const nav_t *nav,
                 const double *rr, const prcopt_t *opt, int index, double *y,
                 double *e, double *azel)
{
	double r,rr_[3],pos[3];
	double zhd, zazel[]={0.0,90.0*D2R};//tai sao init zazel = {0,90}
	int i;
	
	for (i=0;i<2*n;i++) y[i]=0.0;
	if (sos3(rr)<=0.0) return -1;//no receiver pos
	for (i=0;i<3;i++) rr_[i]=rr[i];
	ecef2pos(rr_,pos);

																						
	for (i=0;i<n;i++)
	{
		if ((r=geodist(rs+i*6,rr_,e+i*3))<=0.0) continue;
		if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;
		//satellite clock-bias
		r -= CLIGHT*dts[i*2];
    /* troposphere delay model (hydrostatic) */
    zhd=tropmodel(obs[0].time,pos,zazel,0.0);//tai sao chon rel_humi = 0???????????
    r+=tropmapf(obs[i].time,pos,azel+i*2,NULL)*zhd;
    /* undifferenced phase/code residual for satellite */
    zdres_sat(base,r,obs+i,nav,azel+i*2,opt,y+i*2);
	}
	return 0;
}
//0:ok, !0:not ok
static int validobs(int i, int j, int f, int nf, double *y)
{
    /* if no phase observable, psudorange is also unusable */
    return !(y[f+i*nf*2]!=0.0&&y[f+j*nf*2]!=0.0&&
           (f<nf||(y[f-nf+i*nf*2]!=0.0&&y[f-nf+j*nf*2]!=0.0)));
}
static void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                  int nv, double *R)
{
    int i,j,k=0,b;
    
    for (i=0;i<nv*nv;i++) R[i]=0.0;
    for (b=0;b<n;k+=nb[b++]) {
        
        for (i=0;i<nb[b];i++) for (j=0;j<nb[b];j++) {
            R[k+i+(k+j)*nv]=Ri[k+i]+(i==j?Rj[k+i]:0.0);
        }
    }
}
static int ddres(rtk_t *rtk, const nav_t *nav, double dt, const double *x,
                 const double *P, const int *sat, double *y, double *e,
                 double *azel, const int *iu, const int *ir, int ns, double *v,
                 double *H, double *R, int *vflg,char **msg)
{
    prcopt_t *opt=&rtk->opt;
    double bl,dr[3],posu[3],posr[3],*im;
    double *tropr,*tropu,*dtdxr,*dtdxu,*Ri,*Rj,s,lami,lamj,*Hi=NULL;
    int i,j,k,f,nv=0,nb[6]={0},b=0,sysi,sysj,nf=1;
    
    bl=baseline(x,rtk->rb,dr);
    ecef2pos(x,posu); ecef2pos(rtk->rb,posr);
		sysi=sysj=SYS_GPS;
    
    Ri=mat(ns*2+2,1); Rj=mat (ns*2+2,1); im=mat(ns,1);
    tropu=mat(ns,1); tropr=mat(ns,1); dtdxu=mat(ns,3); dtdxr=mat(ns,3);
    
    for (i=0;i<MAX_SAT;i++) {
        rtk->ssat[i].resp=rtk->ssat[i].resc=0.0;
    }
    
    for (f=0;f<2;f++) 
		{
        /* search reference satellite with highest elevation */
        for (i=-1,j=0;j<ns;j++) {    
            
            if (validobs(iu[j],ir[j],f,1,y)) continue;
            if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
        }
        if (i<0) continue;
        
        /* make double difference */
        for (j=0;j<ns;j++) {
            if (i==j) continue;
            if (validobs(iu[j],ir[j],f,1,y)) continue;
            
						lami=lamj=LAM1;
            if (H) Hi=H+nv*rtk->nx;
            
            /* double-differenced residual */
            v[nv]=(y[f+iu[i]*2]-y[f+ir[i]*2])-
                  (y[f+iu[j]*2]-y[f+ir[j]*2]);
            
            /* partial derivatives by rover position */
            if (H) {
                for (k=0;k<3;k++) {
                    Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];
                }
            }
 
            /* double-differenced phase-bias term */
            if (f<1) {
                v[nv]-=lami*x[IB(sat[i],f,opt)]-lamj*x[IB(sat[j],f,opt)];
                if (H)
								{
									Hi[IB(sat[i],f,opt)]= lami;
                  Hi[IB(sat[j],f,opt)]=-lamj;
 								}
            }

            if (f<1) rtk->ssat[sat[j]-1].resc=v[nv];
            else      rtk->ssat[sat[j]-1].resp=v[nv];
            
            /* test innovation */
            if (opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
                if (f<1) {
                    rtk->ssat[sat[i]-1].rejc++;
                    rtk->ssat[sat[j]-1].rejc++;
                }
                (*msg)+=sprintf(*msg,"outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
                       sat[i],sat[j],f<1?"L":"P",f+1,v[nv]);
                continue;
            }
            /* single-differenced measurement error variances */
            Ri[nv]=varerr(sat[i],sysi,azel[1+iu[i]*2],bl,dt,f,opt);
            Rj[nv]=varerr(sat[j],sysj,azel[1+iu[j]*2],bl,dt,f,opt);
            
            /* set valid data flags */
            
            if (f<1) rtk->ssat[sat[i]-1].vsat=rtk->ssat[sat[j]-1].vsat=1;
            
 
            //trace(4,"sat=%3d-%3d %s%d v=%13.3f R=%8.6f %8.6f\n",sat[i],
            //      sat[j],f<nf?"L":"P",f%nf+1,v[nv],Ri[nv],Rj[nv]);
            
            vflg[nv++]=(sat[i]<<16)|(sat[j]<<8)|((f<nf?0:1)<<4);
            nb[b]++;
        }
        /* restore single-differenced residuals assuming sum equal zero */
        if (f<nf) {
            for (j=0,s=0.0;j<MAX_SAT;j++) s+=rtk->ssat[j].resc;
            s/=nb[b]+1;
            for (j=0;j<MAX_SAT;j++) {
                if (j==sat[i]-1||rtk->ssat[j].resc!=0.0) rtk->ssat[j].resc-=s;
            }
        }
        else {
            for (j=0,s=0.0;j<MAX_SAT;j++) s+=rtk->ssat[j].resp;
            s/=nb[b]+1;
            for (j=0;j<MAX_SAT;j++) {
                if (j==sat[i]-1||rtk->ssat[j].resp!=0.0)
                    rtk->ssat[j].resp-=s;
            }
        }
        b++;
    }
    /* end of system loop */
    
    /* baseline length constraint for moving baseline */

    /* double-differenced measurement error covariance */
    ddcov(nb,b,Ri,Rj,nv,R);
    
    free(Ri); free(Rj); free(im);
    free(tropu); free(tropr); free(dtdxu); free(dtdxr);
    
    return nv;
}
/* single to double-difference transformation matrix (D') --------------------*/
static int ddmat(rtk_t *rtk, double *D,char **msg)
{
    int i,j,k,f,nb=0,nx=rtk->nx,na=rtk->na,nf=NF(&rtk->opt);
    //*msg += sprintf(*msg,"A");
    for (i=0;i<MAX_SAT;i++){
        rtk->ssat[i].fix=0;
    }
		//*msg += sprintf(*msg,"B");
    for (i=0;i<na;i++) D[i+i*nx]=1.0;
    //*msg += sprintf(*msg,"C");
		for (f=0,k=na;f<nf;f++,k+=MAX_SAT) {
				for (i=k;i<k+MAX_SAT;i++) {
						if ((rtk->x[i]==0.0)||(!rtk->ssat[i-k].vsat)) 
								continue;
						
						if (rtk->ssat[i-k].lock>0&&!(rtk->ssat[i-k].slip&2)&&
								rtk->ssat[i-k].azel[1]>=rtk->opt.elmaskar) {
								rtk->ssat[i-k].fix=2; /* fix */
								break;
						}
						else rtk->ssat[i-k].fix=1;
				}
				for (j=k;j<k+MAX_SAT;j++) {
						if (i==j||rtk->x[j]==0.0||
                    !rtk->ssat[j-k].vsat) 
								continue;
						
						if (rtk->ssat[j-k].lock>0&&!(rtk->ssat[j-k].slip&2)&&
								rtk->ssat[i-k].vsat&&
								rtk->ssat[j-k].azel[1]>=rtk->opt.elmaskar) {
								D[i+(na+nb)*nx]= 1.0;
								D[j+(na+nb)*nx]=-1.0;
								nb++;
								rtk->ssat[j-k].fix=2; /* fix */
						}
						else rtk->ssat[j-k].fix=1;
				}
		}
    
    //*msg += sprintf(*msg,"D");
		return nb;
}

static void restamb(rtk_t *rtk, const double *bias, int nb, double *xa)
{
    int i,n,f,index[MAX_SAT],nv=0;
    
    for (i=0;i<rtk->nx;i++) xa[i]=rtk->x [i];
    for (i=0;i<rtk->na;i++) xa[i]=rtk->xa[i];
    f=0;
		for (n=i=0;i<MAX_SAT;i++) {
				if (rtk->ssat[i].fix!=2) continue;
				index[n++]=IB(i+1,f,&rtk->opt);
		}
		if (n>=2) 
		{
			xa[index[0]]=rtk->x[index[0]];
			
			for (i=1;i<n;i++) {
					xa[index[i]]=xa[index[0]]-bias[nv++];
			}
		}
}
/* hold integer ambiguity ----------------------------------------------------*/
static void holdamb(rtk_t *rtk, const double *xa,char **msg)
{
    double *v,*H,*R;
    int i,n,f,info,index[MAX_SAT],nb=rtk->nx-rtk->na,nv=0;
    
    v=mat(nb,1); H=zeros(nb,rtk->nx);
    
        for (n=i=0;i<MAX_SAT;i++) {
            if (rtk->ssat[i].fix!=2||
                rtk->ssat[i].azel[1]<rtk->opt.elmaskhold) continue;
            
            index[n++]=IB(i+1,f,&rtk->opt);
            rtk->ssat[i].fix=3; /* hold */
        }
        /* constraint to fixed ambiguity */
        for (i=1;i<n;i++) {
            v[nv]=(xa[index[0]]-xa[index[i]])-(rtk->x[index[0]]-rtk->x[index[i]]);
            
            H[index[0]+nv*rtk->nx]= 1.0;
            H[index[i]+nv*rtk->nx]=-1.0;
            nv++;
        }
    
    if (nv>0) {
        R=zeros(nv,nv);
        for (i=0;i<nv;i++) R[i+i*nv]=VAR_HOLDAMB;
        
        /* update states with constraints */
        if ((info=filter(rtk->x,rtk->P,H,v,R,rtk->nx,nv))) {
            (*msg)+=sprintf(*msg,"filter error (info=%d)\n",info);
        }
        free(R);
    }
    free(v); free(H);
}
/* resolve integer ambiguity by LAMBDA ---------------------------------------*/
static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa,char **msg)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,ny,nb,info,nx=rtk->nx,na=rtk->na;
    double *D,*DP,*y,*Qy,*b,*db,*Qb,*Qab,*QQ,s[2];
    
    rtk->sol.ratio=0.0;

    /* single to double-difference transformation matrix (D') */
    D=zeros(nx,nx);
		if (!D)
		{
			*msg+=sprintf(*msg,"resamb_LAMBDA mat error1\n");
			return 0;
		}
    if ((nb=ddmat(rtk,D,msg))<=0) {
        *msg+=sprintf(*msg,"no valid double-difference\n");
        free(D);
        return 0;
    }
    ny=na+nb; y=mat(ny,1); Qy=mat(ny,ny); DP=mat(ny,nx);
    b=mat(nb,2); db=mat(nb,1); Qb=mat(nb,nb); Qab=mat(na,nb); QQ=mat(na,nb);
		if ((!y)||(!Qy)||(!DP)||(!b)||(!db)||(!Qb)||(!Qab)||(!QQ))
		{
			*msg+=sprintf(*msg,"resamb_LAMBDA mat error2\n");
			free(D);free(y);free(Qy);free(DP);free(b);free(db);
			free(Qb);free(Qab);free(QQ);
			return 0;
    }
    /* transform single to double-differenced phase-bias (y=D'*x, Qy=D'*P*D) */
    matmul("TN",ny, 1,nx,1.0,D ,rtk->x,0.0,y );
    matmul("TN",ny,nx,nx,1.0,D ,rtk->P,0.0,DP);
    matmul("NN",ny,ny,nx,1.0,DP,D     ,0.0,Qy);
    
    /* phase-bias covariance (Qb) and real-parameters to bias covariance (Qab) */
    for (i=0;i<nb;i++) for (j=0;j<nb;j++) Qb [i+j*nb]=Qy[na+i+(na+j)*ny];
    for (i=0;i<na;i++) for (j=0;j<nb;j++) Qab[i+j*na]=Qy[   i+(na+j)*ny];
    
    /* lambda/mlambda integer least-square estimation */
    if (!(info=lambda(nb,2,y+na,Qb,b,s))) {
               
        rtk->sol.ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
        if (rtk->sol.ratio>999.9) rtk->sol.ratio=999.9f;
        
        /* validation by popular ratio-test */
        if (s[0]<=0.0||s[1]/s[0]>=opt->thresar[0]) {
            
            /* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b)) */
            for (i=0;i<na;i++) {
                rtk->xa[i]=rtk->x[i];
                for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
            }
            for (i=0;i<nb;i++) {
                bias[i]=b[i];
                y[na+i]-=b[i];
            }
            if (!matinv(Qb,nb)) {
                matmul("NN",nb,1,nb, 1.0,Qb ,y+na,0.0,db);
                matmul("NN",na,1,nb,-1.0,Qab,db  ,1.0,rtk->xa);
                
                /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
                matmul("NN",na,nb,nb, 1.0,Qab,Qb ,0.0,QQ);
                matmul("NT",na,na,nb,-1.0,QQ ,Qab,1.0,rtk->Pa);
                
 
                
                /* restore single-differenced ambiguity */
                restamb(rtk,bias,nb,xa);
            }
            else nb=0;
        }
        else { /* validation failed */
            (*msg)+=sprintf(*msg,"ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                   nb,s[1]/s[0],s[0],s[1]);
            nb=0;
        }
    }
    else {
        (*msg)+=sprintf(*msg,"lambda error (info=%d)\n",info);
    }
    free(D); free(y); free(Qy); free(DP);
    free(b); free(db); free(Qb); free(Qab); free(QQ);
    
    return nb; /* number of ambiguities */
}
//0:ok,!0:not ok
static int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
                  int nv, double thres,char **msg)
{
#if 0
    prcopt_t *opt=&rtk->opt;
    double vv=0.0;
#endif
    double fact=thres*thres;
    int i,stat=1,sat1,sat2,type,freq;
    char *stype;
    
    /* post-fit residual test */
    for (i=0;i<nv;i++) {
        if (v[i]*v[i]<=fact*R[i+i*nv]) continue;
        sat1=(vflg[i]>>16)&0xFF;
        sat2=(vflg[i]>> 8)&0xFF;
        type=(vflg[i]>> 4)&0xF;
        freq=vflg[i]&0xF;
        stype=type==0?"L":(type==1?"L":"C");
				//WARNING: msg length may be greater than MAX_ERRMSG
        //(*msg)+=sprintf(*msg,"large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
				//      sat1,sat2,stype,freq+1,v[i],SQRT(R[i+i*nv]));
    }
#if 0 /* omitted v.2.4.0 */
    if (stat&&nv>NP(opt)) {
        
        /* chi-square validation */
        for (i=0;i<nv;i++) vv+=v[i]*v[i]/R[i+i*nv];
        
        if (vv>chisqr[nv-NP(opt)-1]) {
            errmsg(rtk,"residuals validation failed (nv=%d np=%d vv=%.2f cs=%.2f)\n",
                   nv,NP(opt),vv,chisqr[nv-NP(opt)-1]);
            stat=0;
        }
        else {
            trace(3,"valpos : validation ok (%s nv=%d np=%d vv=%.2f cs=%.2f)\n",
                  rtk->tstr,nv,NP(opt),vv,chisqr[nv-NP(opt)-1]);
        }
    }
#endif
    return !stat;
}


//0:ok, !0: not ok
static int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                  const nav_t *nav,char **msg)
{
  prcopt_t *opt=&rtk->opt;
  gtime_t time=obs[0].time;
  double *rs,*dts,*var,*y,*e,*azel,*v,*H,*R,*bias,dt;
	double *xp,*Pp,*xa;
  int i,n=nu+nr,ns,ny,nv,sat[MAX_SAT],iu[MAX_SAT],ir[MAX_SAT];
  int info,vflg[MAX_OBS*2+1],svh[MAX_OBS*2];
  int stat=SOLQ_FLOAT;

  dt=timediff(time,obs[nu].time);	
  rs=mat(6,n); dts=mat(2,n); var=mat(1,n); y=mat(2,n); e=mat(3,n);
  azel=zeros(2,n);
	if ((!rs)||(!dts)||(!var)||(!y)||(!e)||(!azel))
	{	
		(*msg)+=sprintf(*msg,"relpos mat error 1\n");		
		free(rs); free(dts); free(var); free(y); free(e); free(azel);
		return -1;
	}
	for (i=0;i<MAX_SAT;i++)
	{
		rtk->ssat[i].vsat = rtk->ssat[i].snr=0;
	}
  /* satellite positions/clocks*/
	satposs(time,obs,n,nav,rs,dts,var,svh,msg);	//da tinh trong pntpos rui**************************
	/* undifferenced residuals for base station */
	if (zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,svh+nu,nav,rtk->rb,opt,1,
               y+nu*2,e+nu*3,azel+nu*2)) //*****************************************
	{
    (*msg)+=sprintf(*msg,"initial base station position error\n");
		
    free(rs); free(dts); free(var); free(y); free(e); free(azel);
    return -1;
	}//*********************88
	/* select common satellites between rover and base-station */
  if ((ns=selsat(obs,azel,nu,nr,opt,sat,iu,ir,msg))<=0) {//check!!!!!!!!!!!!!!!*************************************
		
    (*msg)+=sprintf(*msg,"no common satellite\n");
    free(rs); free(dts); free(var); free(y); free(e); free(azel);
    return -1;
  }
//	(*msg)+=sprintf(*msg,"common satellite:%d\n",ns);
	/* temporal update of states *///***************************************************************************
  udstate(rtk,obs,sat,iu,ir,ns,nav,msg);//*********************************************
	
	xp=mat(rtk->nx,1); Pp=zeros(rtk->nx,rtk->nx); xa=mat(rtk->nx,1);
	matcpy(xp,rtk->x,rtk->nx,1);
	
	ny=ns*2+2;
	v=mat(ny,1); H=zeros(rtk->nx,ny); R=mat(ny,ny); bias=mat(rtk->nx,1);
	if ((!xp)||(!Pp)||(!xa)||(!v)||(!H)||(!R)||(!bias))
	{	
		(*msg)+=sprintf(*msg,"relpos mat error 2\n");
		free(rs); free(dts); free(var); free(y); free(e); free(azel);
		free(xp); free(Pp);  free(xa);  free(v); free(H); free(R); free(bias);
		return -1;
	}
	
	//*************************************************8
	for (i=0;i<NITER;i++) {
		/* undifferenced residuals for rover */
		if (zdres(0,obs,nu,rs,dts,svh,nav,xp,opt,0,y,e,azel)) {
			(*msg)+=sprintf(*msg,"rover initial position error\n");
			stat=SOLQ_NONE;
			break;
		}
		//*************************
		/* double-differenced residuals and partial derivatives */
		if ((nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,iu,ir,ns,v,H,R,vflg,msg))<1) {
			(*msg)+=sprintf(*msg,"no double-differenced residual\n");
			stat=SOLQ_NONE;
			break;
		}
		/* kalman filter measurement update */
		matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
		if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv))) {
			(*msg)+=sprintf(*msg,"filter error (info=%d)\n",info);
			stat=SOLQ_NONE;
			break;
		}
		
	}
	//***************************
	if (stat!=SOLQ_NONE&&(!zdres(0,obs,nu,rs,dts,svh,nav,xp,opt,0,y,e,azel))) {

		/* post-fit residuals for float solution */
		nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,iu,ir,ns,v,NULL,R,vflg,msg);

		/* validation of float solution */
		if (!valpos(rtk,v,R,vflg,nv,4.0,msg)) {//0:ok,!0:not ok
			

			/* update state and covariance matrix */
			matcpy(rtk->x,xp,rtk->nx,1);
			matcpy(rtk->P,Pp,rtk->nx,rtk->nx);

			/* update ambiguity control struct */
			rtk->sol.ns=0;
			for (i=0;i<ns;i++)
			{
				if (!rtk->ssat[sat[i]-1].vsat) continue;
				rtk->ssat[sat[i]-1].lock++;
				rtk->ssat[sat[i]-1].outc=0;
				rtk->sol.ns++; /* valid satellite count by L1 */
			}
			/* lack of valid satellites */
			if (rtk->sol.ns<4) stat=SOLQ_NONE;
		}
		else stat=SOLQ_NONE;
	}

	/* resolve integer ambiguity by LAMBDA */
	if (stat!=SOLQ_NONE&&(resamb_LAMBDA(rtk,bias,xa,msg)>1)) {
		
		if (!zdres(0,obs,nu,rs,dts,svh,nav,xa,opt,0,y,e,azel)) {

			/* post-fit reisiduals for fixed solution */
			nv=ddres(rtk,nav,dt,xa,NULL,sat,y,e,azel,iu,ir,ns,v,NULL,R,vflg,msg);

			/* validation of fixed solution */
			if (!valpos(rtk,v,R,vflg,nv,4.0,msg)) {

				/* hold integer ambiguity */
				if (++rtk->nfix>=rtk->opt.minfix&&
					rtk->opt.modear==ARMODE_FIXHOLD) {
					holdamb(rtk,xa,msg);
				}
				stat=SOLQ_FIX;
			}
		}
	}
	/* save solution status */
	if (stat==SOLQ_FIX) {
		for (i=0;i<3;i++) {
			rtk->sol.rr[i]=rtk->xa[i];
			rtk->sol.qr[i]=(float)rtk->Pa[i+i*rtk->na];
		}
		rtk->sol.qr[3]=(float)rtk->Pa[1];
		rtk->sol.qr[4]=(float)rtk->Pa[1+2*rtk->na];
		rtk->sol.qr[5]=(float)rtk->Pa[2];
	}
	else {
		for (i=0;i<3;i++) {
			rtk->sol.rr[i]=rtk->x[i];
			rtk->sol.qr[i]=(float)rtk->P[i+i*rtk->nx];
		}
		rtk->sol.qr[3]=(float)rtk->P[1];
		rtk->sol.qr[4]=(float)rtk->P[1+2*rtk->nx];
		rtk->sol.qr[5]=(float)rtk->P[2];
		rtk->nfix=0;
	}
	for (i=0;i<n;i++){
		if (obs[i].L==0.0) continue;
		rtk->ssat[obs[i].sat-1].pt[obs[i].rcv-1]=obs[i].time;
		rtk->ssat[obs[i].sat-1].ph[obs[i].rcv-1]=obs[i].L;
	}
	for (i=0;i<ns;i++){

		/* output snr of rover receiver */
		rtk->ssat[sat[i]-1].snr=obs[iu[i]].SNR;
	}
	for (i=0;i<MAX_SAT;i++){
		if (rtk->ssat[i].fix==2&&stat!=SOLQ_FIX) rtk->ssat[i].fix=1;
		if (rtk->ssat[i].slip&1) rtk->ssat[i].slipc++;
	}
	free(rs); free(dts); free(var); free(y); free(e); free(azel);
	free(xp); free(Pp);  free(xa);  free(v); free(H); free(R); free(bias);

	if (stat!=SOLQ_NONE) rtk->sol.stat=stat;
	
	return stat==SOLQ_NONE;	
}
void rtkinit(rtk_t *rtk,const prcopt_t *opt)
{
	gtime_t time0={0};
	ambc_t ambc0={{0}};
	ssat_t ssat0;
	double *x0=rtk->x;
	double *P0=rtk->P;
	double *xa=rtk->xa;
	double *Pa=rtk->Pa;
	char *errmsg=rtk->errbuf;
	int i,j,tmp;
	double pos[3];
	rtk->opt=*opt;
	rtk->nx=NX(opt);
  rtk->na=NR(opt);
	//base position
#ifdef BASE_POS_TYPE_LLH
	pos[0]=opt->rb[0]*D2R;
	pos[1]=opt->rb[1]*D2R;
	pos[2]=opt->rb[2];
	pos2ecef(pos,rtk->rb);
#else	
	for (i=0;i<3;i++)
		rtk->rb[i]=opt->rb[i];
#endif

	//base velocity
	for (i=3;i<6;i++)
	{
		rtk->rb[i] = 0.0;
	}
	//rover position and velocities
	for (i=0;i<6;i++)
		rtk->sol.rr[i]=0.0;
	rtk->sol.dtr[0]=0.0;
	rtk->sol.time=time0;
	
	rtk->tt = 0.0;//time diff
	for (i=0,tmp=0;i<rtk->nx;i++)
	{
		x0[i]=0.0;
		for (j=0;j<rtk->nx;j++)
		{
			P0[tmp++]=0.0;
		}
	}
	for (i=0,tmp=0;i<rtk->na;i++)
	{
		xa[i]=0.0;
		for (j=0;j<rtk->na;j++)
		{
			Pa[tmp++]=0.0;
		}
	}
	rtk->nfix=rtk->neb=0;
	for (i=0;i<MAX_SAT;i++)
	{
		rtk->ambc[i]=ambc0;
		rtk->ssat[i]=ssat0;
	}
	for (i=0;i<MAX_ERRMSG;i++)
	{
		errmsg[i]=0;
	}	
}
//return: 0 ok, !0 error
int rtkpos(rtk_t *rtk,const obsd_t *obs, int n, const nav_t *nav)
{
	gtime_t time;
	int i,nu,nr;
	int tmp;
	double ep[6];
	prcopt_t *opt=&rtk->opt;
	char *errMsg=rtk->errbuf;//MAKE SURE errMsg length is less than MAX_ERRMSG
	rtk->errLen = 0;
//	char **msg=&errMsg;
	for (i=0;i<n;i++)
	{
		if (((obs+i)->rcv) != 1)
			break;
	}

	nu=i;//number of rover observations
	nr=n-nu;
	time=rtk->sol.time;//previous epoch
/*	time2epoch(time,ep);
	if ((ep[4]>=6.0) && (ep[5]>=19.0))
	{
		errMsg +=sprintf(errMsg,"die");
	}
*/
	
	//rover standard positioning
	if (pntpos(obs,nu,nav,&rtk->sol,NULL,rtk->ssat,&rtk->opt,&errMsg))	
	{
		rtk->sol.stat=SOLQ_NONE;
		rtk->errLen = errMsg-rtk->errbuf;
		return -1;
	}
	if (time.time !=0)//first solution
		rtk->tt=timediff(rtk->sol.time,time);
	if (rtk->opt.mode==PMODE_SINGLE)
	{
		rtk->sol.stat=SOLQ_SINGLE;
		return 0;
	}
	if (nr==0)
	{
		errMsg+=sprintf(errMsg,"no base obs data\n");
		rtk->errLen = errMsg-rtk->errbuf;
		rtk->sol.stat=SOLQ_SINGLE;
//		return 0;
		return -1;
	}
	//check age of differential
  rtk->sol.age=(float)timediff(obs[0].time,obs[nu].time);
  if ((rtk->sol.age>opt->maxtdiff) || (rtk->sol.age<-opt->maxtdiff)) 
	{
    errMsg+=sprintf(errMsg,"age of differential error (age=%.1f)\n",rtk->sol.age);
		rtk->errLen = errMsg-rtk->errbuf;
//    rtk->sol.stat=SOLQ_SINGLE;
    return -1;
  }	
	
	tmp = relpos(rtk,obs,nu,nr,nav,&errMsg);
	rtk->errLen = errMsg-(rtk->errbuf);
	
	return tmp;	
	
}

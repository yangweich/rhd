 /** \file profibus.c
 *  \ingroup hwmodule
 *
 *   Interface for Guidebot Profibus Interface
 *
 * This file implements the profibus library interface for controlling the
 * guidebot.
 *
 ***************************************************************************/
/*********************** Version control information ***********************/
 #define REVISION         "$Rev: 59 $:"
 #define DATE             "$Date: 2011-07-02 06:52:23 +0200 (Sat, 02 Jul 2011) $:"
 #define ID               "$Id: profibus.c 59 2012-10-21 06:25:02Z jcan $"
/***************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 #include <unistd.h>

#include "profibus.h"
#include "dp_5613.h"

//Enable / disable debugging output (uncomment for debug output)
//#define OUTPUT_DESIRED

/*Constant to divide Analog input values to when input is +/-10V*/
#define AD_CONST 3275
/*Number of samples for motors to run before acknowledging fault*/
#define FAULTSAMP 5

#if 0
#include <sched.h>
#define iodelay sched_yield
#else
#include <time.h>
void iodelay(void) {
  struct timespec t;
  t.tv_sec = 0;
  t.tv_nsec = 1000;
  nanosleep(&t, 0);
}
#endif

/*
+-------------------------------------------------------------------+
| structures									    |
+-------------------------------------------------------------------+
*/

/* structure for far pointer */
struct CP5613_FP
{
   unsigned long  offset;
   unsigned short selector;
};

/*
+-------------------------------------------------------------------+
| defines										|
+-------------------------------------------------------------------+
*/

#define GBRIGHT 4 /*PROFIBUS address of right hand motor*/
#define GBLEFT 3 /*PROFIBUS address of left hand motor*/
#define GBET200 5 /*PROFIBUS address of ET200S IO module*/
#define GB_maxspeed 0.314 /*Maximum speed of GuideBot*/
#ifdef CI_DPMI
#define CI_FAR far
#else
#define CI_FAR
#endif

#ifdef OUTPUT_DESIRED
#define MYPRINTF(a,b)	   printf(a"\n",b)
#else
#define MYPRINTF(a,b)
#endif

/*
+-------------------------------------------------------------------+
| global data									    |
+-------------------------------------------------------------------+
*/
/*data structs*/
struct pbout profibus_out;
struct pbin profibus_in;

/**/
static DPR_DWORD       user_handle;
static DPR_CP5613_DP_T volatile *dpr_ptr;
static DPR_CP5613_DP_T volatile CI_FAR *dpr_far_ptr;
static DP_ERROR_T      error;

/* Start-up profibus card.
 Code provided by Siemens*/
int profibus_start(void)
{
DPR_DWORD ret;

/*DPR_STRING errtxt[DP_ERR_TXT_SIZE];*/



ret=DP_start_cp("CP5613_1","guidebot.ldb",&error);
if (ret != DP_OK) return -1;

MYPRINTF("DP_start_cp=%lx",ret);

/*ret=DP_get_err_txt(&error,"English", errtxt);
MYPRINTF("DP_get_err=%lx",ret);
rt_vga_text_n(4,15,0,DP_ERR_TXT_SIZE,errtxt);*/


ret=DP_open("CP5613_1",&user_handle,&error);
if (ret != DP_OK) return -1;

MYPRINTF("DP_open=%lx",ret);
   
ret=DP_get_pointer(user_handle,DP_TIMEOUT_FOREVER,&dpr_ptr,&error);
if (ret != DP_OK) return -1;
      
	 /* DP interface has been compiled in flat modell,	 */
	 /* pointer to DPRAM must be far pointer.		 */
	 /* So dpr_ptr is near pointer to far pointer, where */
	 /* the address of the DPRAM is stored in.		 */
#ifdef CI_DPMI
	 *((struct CP5613_FP far *)&dpr_far_ptr)=
		*((struct CP5613_FP far *)dpr_ptr);
#else
	 dpr_far_ptr=dpr_ptr;
#endif
switch(dpr_far_ptr->info_watch.master_info.USIF_state)
	 {
	 case DP_OFFLINE:
		{
		   ret=DP_set_mode(user_handle,DP_STOP,&error);
			if (ret != DP_OK) return -1;
		   MYPRINTF("DP_set_mode(DP_STOP)=%lx",ret);
		   do
		   {
		   }  while(dpr_far_ptr->info_watch.master_info.USIF_state!=DP_STOP);
		   /* fall through */
		}
	 case DP_STOP:
		{
		   ret=DP_set_mode(user_handle,DP_CLEAR,&error);
			if (ret != DP_OK) return -1;
		   MYPRINTF("DP_set_mode(DP_CLEAR)=%lx",ret);
		   do
		   {
		   }  while(dpr_far_ptr->info_watch.master_info.USIF_state!=DP_CLEAR);
		   /* fall through */
		}
	 case DP_CLEAR:
		{
		   ret=DP_set_mode(user_handle,DP_OPERATE,&error);
			if (ret != DP_OK) return -1;
		   MYPRINTF("DP_set_mode(DP_OPERATE)=%lx",ret);
		   do
		   {
		   }  while(dpr_far_ptr->info_watch.master_info.USIF_state!=DP_OPERATE);
		   /* fall through */
		}
	 }

return 1;
	 
}


/* Shut-down profibus card.
Code provided by Siemens */
void profibus_stop(void)
{
DPR_DWORD ret;
 	 ret=DP_close(user_handle,&error);
   MYPRINTF("DP_close=%lx",ret);
   
   ret=DP_reset_cp("CP5613_1",&error);
   MYPRINTF("DP_reset_cp=%lx",ret);
}


/* Send request telegram to motors
 */
void profibus_sensors_command(void)
{
unsigned char outbuf[12], etbuf[18];


/*Compose encoder request telegram, empty telegram if fault*/
outbuf[0]=0x10*!profibus_in.lfault;
outbuf[1]=0x28*!profibus_in.lfault;
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x00;
outbuf[8]=0x75*!profibus_in.lfault; /*Bytes 8 and 9 need to be set to keep the motors in jog mode*/
outbuf[9]=0xff*!profibus_in.lfault;
outbuf[10]=0x00;
outbuf[11]=0x00;


if (profibus_out.lspeed<0)
	{
	outbuf[8]=0x76*!profibus_in.lfault; /*Jog the other way if speed is negative*/
	}


/*Send request to left POSMO*/
memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
/*Wait for change to be committed*/
iodelay();
iodelay();
iodelay();
/*Compose encoder request telegram*/
outbuf[0]=0x10*!profibus_in.rfault;
outbuf[1]=0x28*!profibus_in.rfault;
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x00;
outbuf[8]=0x76*!profibus_in.rfault; /*Bytes 8 and 9 need to be set to keep the motors in jog mode*/
outbuf[9]=0xff*!profibus_in.rfault;
outbuf[10]=0x00;
outbuf[11]=0x00;
	
if (profibus_out.rspeed<0)
	{
	outbuf[8]=0x75*!profibus_in.rfault;
	}

/*Send request to right POSMO*/
memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;
iodelay();
iodelay();
iodelay();
etbuf[0]=profibus_out.signal_state;
/*Send request to ET200S*/
memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBET200].data[0]),etbuf,1);
dpr_far_ptr->ctr.D_out_slave_adr=GBET200;

}


/* Timeout profibus data collection.
 */
void profibus_sensors_read(void)
{
unsigned char lbuf[12],rbuf[12],et200[26];
DPR_INT16 jsp,jdiff,batt;
DPR_WORD lstate,rstate;
DPR_INT32 lenc,renc;
//unsigned long long time1,time2;
//unsigned int dt,temp;
	

	/*Read from ET200S*/
	
	
		dpr_far_ptr->ctr.D_lock_in_slave_adr = GBET200;
		memcpy(et200,(void*)(&dpr_far_ptr->pi.slave_in[GBET200].data[0]),26);
		dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;

	 profibus_in.digi_in[0] = (et200[0]&0x01);
	 profibus_in.digi_in[1] = (et200[0]&0x02)>>1;
 	 profibus_in.digi_in[2] = (et200[0]&0x04)>>2;
 	 profibus_in.digi_in[3] = (et200[0]&0x08)>>3;
 	 profibus_in.digi_in[4] = (et200[0]&0x010)>>4;
 	 profibus_in.digi_in[5] = (et200[0]&0x020)>>5;

	 
	 /*Check if joystick present*/
	 profibus_in.Joystick = (et200[0]&0x01);
	 profibus_in.loglinedata = (et200[0]&0x02);
	 /*Check if obstacle present*/
	 profibus_in.Obstacle =	!(et200[0]&0x04);
		
		/*Joystick values extracted from ET200S telegram*/
	 jsp=et200[6];
	 jsp=jsp << 8;
	 jsp=jsp+et200[7];
	 jsp=jsp/16; /*Shifting 4 bits, keeping sign*/

	 jdiff=et200[8];
	 jdiff=jdiff << 8;
	 jdiff=jdiff+et200[9];
	 jdiff=jdiff/16;	
	 
	 /*Battery voltage extracted from ET200S telegram*/
	 batt=et200[2];
	 batt=batt << 8;
	 batt=batt + et200[3];
	 batt=batt/2; /*Shifting 1 bit, keeping sign*/
	 
	 /*8191 / 10 * 4 / 3.2 = 1023.875*/
	 /*13 bits, 10 Volts, divide by 819 (3276)*/
	 
	 /*Actual value is 3.2 times the measured value (voltage devider 100/220kOhm)*/
	 profibus_in.batt=(float)batt*3.2/819.0;
	 profibus_in.jsp=jsp;
	 profibus_in.jdiff=jdiff;	 	
 

	 /*Read new left motor data*/
	 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
	 memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
	 dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
	 
	 lstate=lbuf[8];
	 lstate=lstate << 8;
	 lstate=lstate+lbuf[9];		   
         //printf("lm %x  %x \n",lbuf[8],lbuf[9]);			
			/*Check if motor has faulted*/
			if(lbuf[9]&0x08) 
				{
				profibus_in.lfault = 1;
				profibus_in.lfaultsamp++;
				}
			else 
				{
				profibus_in.lfault = 0;
				profibus_in.lfaultsamp = 0;
				}
			
			
			/*Display motor status*/
			if(lbuf[9]==0x37)
			{
			profibus_out.lreset=0;
			}
			else 
				if(lbuf[9]==0xb1)
					{
					profibus_out.lreset=0;
					} 
				else
					if(lbuf[8]==0x17)
						{
						profibus_out.lreset=1;
						}
					else
						{
						profibus_out.lreset=1;
						}
			
			
			 	/*left encoder value extracted from telegram*/
			 if (lbuf[1]==0x28) /*Check if telegram received included encoder values*/
			 {
			 	lenc=lbuf[4];
			 	lenc=lenc << 8;
	 		 	lenc=lenc+lbuf[5];
	 		 	lenc=lenc << 8;
	 		 	lenc=lenc+lbuf[6];
	 		 	lenc=lenc << 8;
	 		 	lenc=lenc+lbuf[7];	 
			 	profibus_in.lenc=-lenc; /*1e-5m*/
			 }
			 else
			 {
			 profibus_in.NoLeftEnc++;
			 /*timeout*/
			
			 }	
			 
			 profibus_in.Bumper=(lbuf[9]==0xb1 || lbuf[9]==0xb0);
			 profibus_in.EmStop=(lbuf[8]==0x17);
			 
		  /*	 profibus_in.Sick=(lbuf[9]==0xe0 || lbuf[9]==0xa0 || lbuf[9]==0xf0);*/
	 		
			 /*Read new right encoder data*/
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;

 
			
			rstate=rbuf[8];
			rstate=rstate << 8;
			rstate=rstate+rbuf[9];
			//printf("rm %x  %x \n",rbuf[8],rbuf[9]);	
			/*Check if motor has faulted*/
			if(rbuf[9]&0x08) 
				{
				profibus_in.rfault = 1;
				profibus_in.rfaultsamp++;
				//rt_vga_printf(VGA_Green,12,40,"rfault");
				}
			else 
				{
				profibus_in.rfault = 0;
				profibus_in.rfaultsamp = 0;
				}
			
			if(rbuf[9]==0x37)
			{
			profibus_out.rreset=0;
			}
			else
			if(rbuf[9]==0xb1 || rbuf[9]==0xb3)
			{
			profibus_out.rreset=0;
			} 
			else
				if(rbuf[8]==0x17)
					{
					profibus_out.rreset=1;
					}
				else
					{
					profibus_out.rreset=1;
					} 	
			
			
			 /*right encoder value extracted from telegram*/
				if(rbuf[1]==0x28)
				{
				renc=rbuf[4];	 
	 			renc=renc << 8;
	 			renc=renc+rbuf[5];
				renc=renc << 8;
	 			renc=renc+rbuf[6];
	 			renc=renc << 8;
	 			renc=renc+rbuf[7];
				profibus_in.renc=renc;	 
				}
			else
			 {
			 /*timeout*/
		 	 profibus_in.NoRightEnc++;
			 		
			 }
#if 0
	if (profibus_in.loglinedata && line_sensor.i<DATASTOR)	
				{
				profibus_in.logenc[line_sensor.i][0]=profibus_in.lenc;
	 			profibus_in.logenc[line_sensor.i][1]=profibus_in.renc;
				}
#endif
	

}


/* Profibus motor update.
 */
void profibus_motors_update(void)
{
  /* update motor speeds */
	//char screenbuf[20];
	unsigned char outbuf[12];
	DPR_WORD speed;//,i;
	float rspeed,lspeed;
	//unsigned long long time1,time2;
	//unsigned int dt;

	
	/*Calculate speed as percentage of max*/
	lspeed=(profibus_out.lspeed/GB_maxspeed)*100;
	rspeed=(profibus_out.rspeed/GB_maxspeed)*100;

/*Compose telegram for left motor*/
  outbuf[0]=0x20*!profibus_out.lreset;
	outbuf[1]=0x1a*!profibus_out.lreset;
	outbuf[2]=0x00;
	outbuf[3]=0x00;
	outbuf[4]=0x00;
	outbuf[5]=0x00;
	outbuf[6]=0x00;
	outbuf[7]=0x00;
	outbuf[8]=0x75*!profibus_out.lreset;
	outbuf[9]=0xff*!profibus_out.lreset;
	outbuf[10]=0x00;
	outbuf[11]=0x00;

	if (lspeed<0)
		{
		lspeed=-lspeed;
		outbuf[8]=0x76*!profibus_out.lreset;
		}
	if (lspeed>100)
		{
		lspeed=100;
		}
	/*Calculate what to send to CP*/
	speed=(lspeed/100)*16384;

	outbuf[6]=(speed>>8)*!profibus_out.lreset;
	outbuf[7]=speed*!profibus_out.lreset;

	/*Copy telegram to CP*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	/*Commit changes*/
	dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;

	/*Edit telegram to suit right motor*/
/*Compose telegram for left motor*/
  outbuf[0]=0x20*!profibus_out.rreset;
	outbuf[1]=0x1a*!profibus_out.rreset;

	outbuf[8]=0x76*!profibus_out.rreset;
	outbuf[9]=0xff*!profibus_out.rreset;

	if (rspeed<0)
	{
  rspeed=-rspeed;
	outbuf[8]=0x75*!profibus_out.rreset;
	}
	if (rspeed>100)
	{
	rspeed=100;
	}
	/*Calculate what to send to CP*/
	speed=(rspeed/100)*16384;

	outbuf[6]=(speed>>8)*!profibus_out.rreset;
	outbuf[7]=speed*!profibus_out.rreset;
	iodelay();
	iodelay();
	iodelay();

	/*Copy telegram to CP*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	/*Commit changes*/
	dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

}

void posmo_fault_ack(void)
{
  /* acknowledge faults, reset motors */

	unsigned char outbuf[12];
	outbuf[0]=0x00;
	outbuf[1]=0x00;
	outbuf[2]=0x00;
	outbuf[3]=0x00;
	outbuf[4]=0x00;
	outbuf[5]=0x00;
	outbuf[6]=0x00;
	outbuf[7]=0x00;
	outbuf[8]=0x00;
	outbuf[9]=0x80;
	outbuf[10]=0x00;
	outbuf[11]=0x00;
	

	/*Copy telegram to CP*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	/*Commit changes*/	
	dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
		
	//outbuf[9]=0x00;
	//if (profibus_in.rfaultsamp > FAULTSAMP) outbuf[9]=0x80;
	iodelay();
	iodelay();
	iodelay();


	
	/*Copy telegram to CP*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	/*Commit changes*/
	dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;
	
}

/* Profibus motor parametrization.
 */
void profibus_motors_param(void)
{
 unsigned char lbuf[12],rbuf[12];
 unsigned char temp;
 unsigned char outbuf[12];
 //int i = 0;
 
 /*Wait for profibus to get ready*/
 sleep(3);
 
 
  /* set motor parameters */
	
	/*Parameter 1 is factory preset to 0 which means linear axis*/

	/*Set parameter 2, Travel per shaft revolution. Guidebot has a wheel radius of
	100mm, a belt gear 1:5 and the POSMO'S have a 1:20.25 gear ratio. This means that
	per wheel revolution Guidebot travels 2*100mm*PI = 628.32. Per shaft revolution
	it travels 125.66 mm .This parameter is in the C4 format so it's a double word 
	and the value entered is 10000 times the actual value. This means that the value 
	entered is 1256637 or 0x132CBD in hex.   */

	outbuf[0]=0x30;/*Change a double word parameter*/
	outbuf[1]=0x02;/*Parameter number 2*/
	outbuf[2]=0x00;
	outbuf[3]=0x00;
	outbuf[4]=0x00;/*Most significant parameter byte*/
	outbuf[5]=0x13;
	outbuf[6]=0x2c;
	outbuf[7]=0xbd;/*Least significant parameter byte*/
	outbuf[8]=0x00;/*Last 4 bytes are irrelevant when setting parameters*/
	outbuf[9]=0x00;
	outbuf[10]=0x00;
	outbuf[11]=0x00;

		
		

	//i=0;
	dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
	/*Send request to left POSMO*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	/*Wait for change to be committed*/
	iodelay();	
	iodelay();	

	
	dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
	/*Send request to right POSMO*/
	memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	 
	 
	/*Parameter 3 Gearbox ratio, 20.25 */
	outbuf[0]=0x30;/*Change a double word parameter*/
	outbuf[1]=0x03;/*Parameter number 3*/
	outbuf[2]=0x00;/*Not an array - no sub index*/
	outbuf[3]=0x00;/*Reserved*/
	outbuf[4]=0x01;/*Most significant parameter byte*/
	outbuf[5]=0x34;/*202500 = 0x31704*/ /*20250000=0x134fd90*/
	outbuf[6]=0xfd;/*2025 = 0x7e9*/ /*2025000000=0x78b30c40*/
	outbuf[7]=0x90;/*Least significant parameter byte*/


	dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
	dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;

	//i=0;
		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();	
	 iodelay();	

	
	
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	 

/*Parameter 4 (Dimension units) is factory preset to 0 = mm */
outbuf[0]=0x20;/*Change a word parameter*/
outbuf[1]=0x04;/*Parameter number 4*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x00;/*Least significant parameter byte*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;

	//i=0;
		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();	
	 iodelay();	

	
	
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);

/*Parameters 5-7 are not important for GuideBot*/

/*Parameter 8 (Maximum speed) is factory preset to 3000 RPM 
For GuideBot this means 314mm/s. It is possible to set this speed up to 3300 
RPM which corresponds to 345mm/s*/

/*Paramter 9 (Ramp up time) is factory preset to 100ms*/

/*Parameter 10 (maximum velocity) If we set this according to maximum speed
= 3000RPM * P2 / P3 we get 186.16 mm/min or 0xBA*/


outbuf[0]=0x30;/*Change a double word parameter*/
outbuf[1]=0x0a;/*Parameter number 10*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0xbb;/*Least significant parameter byte*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;


		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();
	 iodelay();	
	 iodelay();	
	 
	
	 
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);


/*Parameters 11-16 are not important for GuideBot*/

/*Parameter 17 (speed controller, P gain) is factory preset to 20 which has yet
to be tested with GuideBot*/

/*Parameter 18 (speed controller, integral action time) is factory preset to
22 ms which has yet to be tested with GuideBot*/

/*Parameters 19-25 will probably not need setting.*/

/*Parameter 26 is set during operation to control speed*/
/*Set it to 0 here for smooth startup*/

outbuf[0]=0x20;/*Change a word parameter*/
outbuf[1]=0x1a;/*Parameter number 26*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x00;/*Least significant parameter byte*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;


		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();
	 iodelay();	
	 iodelay();	
	 
	
	 
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;

	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);


/*Parameters 27-30 will probably not need changing*/

/*Parameter 31 will be set to 100 (OFF1) for inhibit*/
/*Set parameter 31, IO-1 to accept bumper inhibit*/
outbuf[0]=0x20;/*Change a word parameter*/
outbuf[1]=0x1f;/*Parameter number 31*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x64;/*Least significant parameter byte*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;


		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();	
	 iodelay();	
	 iodelay();	
	
	
	
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);

	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;
	
	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	 
#if 1
/*Parameter 32 will be set to  100 (OFF1) for inhibit*/
/*Set parameter 32, IO-2 to accept SICK inhibit*/
outbuf[0]=0x20;/*Change a word parameter*/
outbuf[1]=0x20;/*Parameter number 32*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x64;/*Least significant parameter byte (100) 0x64*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;


		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();	
	 iodelay();	
	 iodelay();	
	
	
	
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;
	 
	
	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	 
#endif

#if 0
/*Write to flash*/
outbuf[0]=0x23;/*Change a word parameter*/
outbuf[1]=0xcb;/*Parameter number 971*/
outbuf[2]=0x00;
outbuf[3]=0x00;
outbuf[4]=0x00;/*Most significant parameter byte*/
outbuf[5]=0x00;
outbuf[6]=0x00;
outbuf[7]=0x01;/*set to 1*/


		dpr_far_ptr->ef.input[3].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		dpr_far_ptr->ef.input[4].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;


		
	 /*Send request to left POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBLEFT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBLEFT;
	 
	 
	 /*Wait for change to be committed*/
	 iodelay();	
	 iodelay();	
	 iodelay();	
	
	
	
	 /*Send request to right POSMO*/
	 memcpy((void*)(&dpr_far_ptr->pi.slave_out[GBRIGHT].data[0]),outbuf,12);
	 dpr_far_ptr->ctr.D_out_slave_adr=GBRIGHT;
	 
	
	 
 temp=0;
	do
		{
		/*Check if new slave data has arrived (parameter has been updated)*/
		if (dpr_far_ptr->ef.input[GBLEFT].req_mask == DPR_DATA_CHANGE)
			 {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBLEFT;
		   memcpy(lbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBLEFT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
		   /*Reset change indicator*/
		  dpr_far_ptr->ef.input[GBLEFT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	temp=0; 
	do
		{
		if (dpr_far_ptr->ef.input[GBRIGHT].req_mask == DPR_DATA_CHANGE)
	
		   {
		   temp=1;
			 dpr_far_ptr->ctr.D_lock_in_slave_adr = GBRIGHT;
		   memcpy(rbuf,(void*)(&dpr_far_ptr->pi.slave_in[GBRIGHT].data[0]),12);
		   dpr_far_ptr->ctr.D_lock_in_slave_adr = 0xffff;
			 dpr_far_ptr->ef.input[GBRIGHT].req_mask = DPR_DATA_INT_CLEAR_AND_MASK;
		   }
		}while(temp!=1);
	 
#endif

}

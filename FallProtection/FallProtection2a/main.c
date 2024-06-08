/*
 * Capstone: Fall Protection Test 2a
 * Author: Keita Yamamoto
 * Date: 4/24/2024
 * Description: Fall Protection Test
 */

/* includes */
#include <stdio.h>
#include <pthread.h>
#include "MyRio.h"
#include "T1.h"
#include "IRQConfigure.h"
#include "TimerIRQ.h"
#include "matlabfiles.h"
#include "Encoder.h"
#include "myLQR1kgc.h"


/* structure and type definition */
typedef struct{
	NiFpga_IrqContext	irqContext;			// IRQ context reserved
	NiFpga_Bool				irqThreadRdy;	// IRQ thread ready flag
} ThreadResource;

typedef struct{
	double	vi;			// initial velocity
	double	xi;			// initial position
	double	amax;		// maximum acceleration
	double	vmax;		// maximum velocity
	double	xf;			// final position
} Param;

typedef struct{
	double	pos;
	double	vel;
	double	acc;
} MStates;

typedef struct{
	double  omega;
	double  force;
	double vel;
	double pos;
} SysStates;

struct biquad{
	double b0; double b1; double b2;		// numerator
	double a0; double a1; double a2;		// denominator
	double x0; double x1; double x2;		// input
	double y1; double y2;							// output
};

/*biquad import */
#include "myLPF.h"

/* prototypes */
void*	Timer_Irq_Thread(void* resource);
void sramp2(Param param, MStates *ref_state, double bti, uint32_t *itime);
double lqr(SysStates states,SysStates ref, double K[], double min, double max);
void getStates(SysStates* states);
double	cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax);

/* definitions */
#define BDI								2000		// encoder counts per revolution, motor side
#define N									5.9			// gear ratio
#define K_MOTOR					.0507		// motor gain
#define K_AMP						.41			// amp gain
#define TMAX							1				// max torque
#define L0									.75		// position offset
#define IMAX							1000		// buffer length
#define SATURATE(x,lo,hi)		((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))		// saturation macro

/*global declarations*/
NiFpga_Session myrio_session;				// myRIO session info
static MyRio_Encoder encC0;
static MyRio_Aio AIC0;
static MyRio_Aio AOC0;

/*-------------------------------------------------------------------------------------------------
 * Function main()
 * Purpose: Create and register interrupt thread, set up s-ramp, set up and calls table editor,
 * 					wait until the editor exits, terminate and unregister interrupt thread
------------------------------------------------------------------------------------------------ */
int main(int argc, char **argv)
{
	NiFpga_Status status;

	status = MyRio_Open();		    			// open FPGA session
	if (MyRio_IsNotSuccess(status)) return status;

    //my code begin
	int32_t isrStatus;									// status for ISR thread setup/termination
	ThreadResource irqThread0;				// thread resource
	MyRio_IrqTimer irqTimer0;					// IRQ channel
	pthread_t thread;									// thread identifier

	int key;													// key pressed

	//specify IRQ channel settings
	irqTimer0.timerWrite = IRQTIMERWRITE;
	irqTimer0.timerSet = IRQTIMERSETTIME;

	//register timer IRQ. Terminate if unsuccessful
	isrStatus = Irq_RegisterTimerIrq(&irqTimer0, &irqThread0.irqContext, timeoutValue);
	if (isrStatus) return isrStatus;

	//ready the interrupt thread resource
	irqThread0.irqThreadRdy = NiFpga_True;

	//create the interrupt thread
	isrStatus = pthread_create(&thread, NULL, Timer_Irq_Thread, &irqThread0);

	//hold until DEL key is pressed
	printf_lcd("\fRunning...");
	while ((key = getkey()) != DEL);

	//terminate ISR thread
	irqThread0.irqThreadRdy = NiFpga_False;
	isrStatus = pthread_join(thread, NULL);

	//unregister timer IRQ
	isrStatus = Irq_UnregisterTimerIrq(&irqTimer0, irqThread0.irqContext);

	printf_lcd("\fProgram End");
	//my code end

	status = MyRio_Close();						// close FPGA session

	return status;
}


/*------------------------------------------------------------------------------------------------
 * Function  Timer_Irq_Thread()
 * Purpose:	Implements  PID control for position of motor.
 * 					Saves  response and other parameters to MATLAB file. Runs on Interrupt thread
 * Arguments:	resource - pointer for thread resource
 * Returns:			NULL
 ------------------------------------------------------------------------------------------------*/
void* Timer_Irq_Thread(void* resource){
	ThreadResource *threadResource = (ThreadResource*) resource;		// casts resource pointer to appropriate structure
	uint32_t irqAssert;																							// IRQ flags

	//fall detection conditions
	NiFpga_Bool fall_detect = NiFpga_False;
	double vel_threshold = -.5;
	double pos_threshold = .5;

	//set up path profile parameters
	Param param;
	param.vi= 0;											// initial velocity (m/s)
	param.xi = 0;											// initial position (m)
	param.vmax = .3;									// maximum velocity (m/s)
	param.amax = 5;								// maximum acceleration (m/s^2)
	param.xf = .5;										// final position (m)

	//set up states
	SysStates ref;
	SysStates states;

	//set up s-ramp
	MStates mass_ref;
	double bti = timeoutValue/1e6;
	uint32_t itime = 0;												// current time index

	//set up controller commands
	double torque;
	double VDAout;

	//set up buffers
	int i;														// index
	double data0[IMAX];							// buffer
	double data1[IMAX];							// buffer
	double data2[IMAX];
	double data3[IMAX];
	double data4[IMAX];
	double data5[IMAX];

	//set up MATLAB
	int	err;														// error flag
	MATFILE *mf;												// MATLAB file

	// AIO initialization
	Aio_InitCI0(&AIC0);									// initialize analog input C 0
	Aio_InitCO0(&AOC0);								// initialize analog output C 0
	Aio_Write(&AOC0,0);								// set AOC0 to 0V

	//Encoder initialization
	EncoderC_initialize(myrio_session,&encC0);

	while (threadResource->irqThreadRdy){
		//wait for interrupt or timeout
		Irq_Wait(threadResource->irqContext,TIMERIRQNO,&irqAssert,(NiFpga_Bool*) &(threadResource->irqThreadRdy));

		//schedule next timer interrupt
		NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, timeoutValue);
		NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True);

		//service and acknowledge IRQ
		if (irqAssert){
			//monitor states
			getStates(&states);

			//check for fall
			if (states.vel < vel_threshold && states.pos < pos_threshold && fall_detect == NiFpga_False ){
				fall_detect = NiFpga_True;
				param.vi = states.vel;
				param.xi = states.pos;

				putchar_lcd(225);
				printf_lcd("\fFall Detected!");
			}

			if (fall_detect){
				//get reference curve
				sramp2(param,&mass_ref,bti,&itime);
				if (i < IMAX) data4[i] = mass_ref.pos;

				//get reference state
				ref.omega = mass_ref.vel/r;
				ref.force = mass_ref.acc*2*m+2*m*9.81;
				ref.vel = mass_ref.vel;
				ref.pos = mass_ref.pos;

				//record states
				if (i < IMAX) data0[i] = states.omega;
				if (i < IMAX) data1[i] = states.force;
				if (i < IMAX) data2[i] = states.vel;
				if (i < IMAX) data3[i] = states.pos;

				// LQR controller
				torque = lqr(states,ref,K,-TMAX,TMAX);
				VDAout = torque/N/K_MOTOR/K_AMP;
				Aio_Write(&AOC0, VDAout);
				if (i < IMAX) data5[i] = VDAout;

				i++;																					// increment i
			}

			// emergency stop
			if (states.vel>10){
				threadResource->irqThreadRdy = NiFpga_False;
				Aio_Write(&AOC0,0);
				putchar_lcd(225);
				printf_lcd("\fEmergency Stop\nVelocity");
			}else if (states.pos >1){
				threadResource->irqThreadRdy = NiFpga_False;
				Aio_Write(&AOC0,0);
				putchar_lcd(225);
				printf_lcd("\fEmergecy Stop\nPosition");
			}

			Irq_Acknowledge(irqAssert);
		}
	}


	//save MATLAB file
	mf = openmatfile("log0528-75.mat",&err);
	if(!mf)	printf_lcd("Can't open mat file %d\n",err);
	matfile_addstring(mf,"myName","Keita Yamamoto");
	matfile_addmatrix(mf,"omega",data0,IMAX,1,0);
	matfile_addmatrix(mf,"force",data1,IMAX,1,0);
	matfile_addmatrix(mf,"vel",data2,IMAX,1,0);
	matfile_addmatrix(mf,"pos",data3,IMAX,1,0);
	matfile_addmatrix(mf,"reference",data4,IMAX,1,0);
	matfile_addmatrix(mf,"VDAout",data5,IMAX,1,0);
	matfile_addmatrix(mf,"BTI",&bti,1,1,0);
	matfile_close(mf);

	//terminate interrupt thread
	pthread_exit(NULL);
	return NULL;
}


/*------------------------------------------------------------------------------------------------
 * Function sramps2()
 * Purpose:	Generates an S-ramp based on initial conditions and the desired final position.
 * Arguments: param		initial conditions and parameters to generate the curve
 * 						 ref_state		reference state of the mass at current time index
 * 						 bti				BTI of the loop
 * 						 itime			time index of the loop, zero for t=0;
 * Returns: 		NONE
 ------------------------------------------------------------------------------------------------*/
void sramp2(Param param, MStates *ref_state, double bti, uint32_t *itime){
	double vi = param.vi;
	double xi = param.xi;
	double amax = param.amax;
	double vmax = param.vmax;
	double xf = param.xf;

	double totalTime;
	double totalDist;
	double accTime;
	double accDist;
	double decelTime ;
	double decelDist;
	double velTime;
	double velDist;


	double pos;
	double vel;
	double acc;

	double t = bti*(*itime)++;


	totalDist = xf-xi;
	accTime = (vmax-vi)/amax;
	accDist = vi*accTime+.5*amax*accTime*accTime;

	decelTime = vmax/amax;
	decelDist = .5*amax*decelTime*decelTime;

	if (totalDist<=accDist+decelDist){												// No constant velocity phase
		decelTime = sqrt((totalDist+(vi*vi/(2*amax)))/amax);
		accTime = decelTime-vi/amax;
		totalTime = accTime+decelTime;
		velTime = 0;
		velDist = 0;
		vmax = 0;
	}else{																								// With constant velocity phase
		velDist = totalDist-accDist-decelDist;
		velTime = velDist/vmax;
		totalTime = accTime+decelTime+velTime;
	}

	if (t<accTime){
	    pos = xi+vi*t+.5*amax*t*t;
	    vel = vi+amax*t;
	    acc = amax;
	}else if (t>=accTime && t<(accTime+velTime)){
	    pos = xi+accDist+vmax*(t-accTime);
	    vel = vmax;
	    acc = 0;
	}else if (t>=(accTime+velTime) && t<totalTime){
	    pos = xf-.5*amax*(totalTime-t)*(totalTime-t);
	    vel = amax*(totalTime-t);
	    acc = -amax;
	}else {
	    pos = xf;
	    vel = 0;
	    acc = 0;
	}

	ref_state->pos = pos;
	ref_state->vel = vel;
	ref_state->acc = acc;
}

/*------------------------------------------------------------------------------------------------
 * Function  lqr()
 * Purpose:
 * Arguments:
 * Returns:
 ------------------------------------------------------------------------------------------------*/
double lqr(SysStates states, SysStates ref, double K[], double min, double max){
	double out = 0;
	int i;

	double error[4] = {ref.omega-states.omega,
									ref.force-states.force,
									ref.vel-states.vel,
									ref.pos-states.pos};

	for (i=0; i<4; i++){
		out = out+error[i]*K[i];
	}
	out  = SATURATE(out,min,max);
	return out;
}

/*------------------------------------------------------------------------------------------------
 * Function  getStates()
 * Purpose:
 * Arguments:
 * Returns:
 ------------------------------------------------------------------------------------------------*/
void getStates(SysStates* states){
	static int first = 1;												// flag for first time running

	int count;
	static int count_ini;
	static int count_prev;
	double theta;
	double omega;
	double vel;
	double vel_filtered;

	double VADin;
	double A =-.05/13;
	double B = A*-6.40;
	double x_spr;
	static double x_spr_prev;
	double v_spr;

	count = Encoder_Counter(&encC0);

	VADin = Aio_Read(&AIC0);
	x_spr = A*VADin+B;

	if (first){																// if getStates() runs for the first time
		count_ini = count;
		count_prev = count;
		 x_spr_prev =x_spr;
		first = 0;
	}

	theta = 2*M_PI*(count-count_ini)/BDI/N;
	omega = 2*M_PI*(count-count_prev)/BDI/N/(timeoutValue*1e-6);
	count_prev = count;

	v_spr = (x_spr-x_spr_prev)/(timeoutValue*1e-6);
	x_spr_prev = x_spr;

	vel = r*omega-2*v_spr;
	vel_filtered = cascade(vel, LPF,LPF_ns, -1000, 1000);

	states->omega = omega;
	states->force = k*x_spr;
	states->pos = L0+r*theta-2*x_spr;
	states->vel =  vel_filtered;
}

/*------------------------------------------------------------------------------------------------
 * Function  cascade()
 * Purpose:	implements biquad cascade for discrete time transfer function
 * Arguments:	xin		- input of transfer function
 * 							fa			- array of biquads
 * 							ns		- number of biquads
 * 							ymin 	- minimum value for transfer function output
 * 							ymax 	- maximum value for transfer function output
 * Returns:			y0		- output of transfer function
 ------------------------------------------------------------------------------------------------*/
double	cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax){
	struct biquad *f;										// pointer for biquads
	double y0;													// output of biquads

	for (f = fa; f < fa+ns; f++){
		if (f == fa){
			f->x0 = xin;										 // first biquad input
		}else{
			f->x0 = y0;											// the other biquad input
		}

		//calculate output of a biquad. Bound the output if it is the final biquad
		y0 = (f->b0*f->x0+f->b1*f->x1+f->b2*f->x2-f->a1*f->y1-f->a2*f->y2)/f->a0;

		//save the values
		 f->x2 = f->x1; f->x1 = f->x0;  f->y2 = f->y1; f->y1 = y0;
	}

	y0 = SATURATE(y0,ymin,ymax);

	return y0;
}

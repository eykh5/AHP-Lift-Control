/*
 * Adaptable House Project: Lift Control Controller
 * Author: Everett Hirano, John Shim, Maya Larson, Keita Yamamoto
 */

/* includes */
#include <stdio.h>
#include <TimerIRQ.h>
#include <unistd.h>
#include <stdlib.h>
#include "MyRio.h"
#include "T1.h"
#include "ctable2.h"
#include "Encoder.h"
#include "pthread.h"
#include "properties.h"


 /*type definition*/
typedef enum {FALL = 0, BWS, FLOAT} State_Type;

struct biquad{
	double b0; double b1; double b2;		// numerator
	double a0; double a1; double a2;		// denominator
	double x0; double x1; double x2;		// input
	double y1; double y2;							// output
};

typedef struct {
NiFpga_IrqContext		irqContext; 			// context
table								*a_table; 					// table
NiFpga_Bool					irqThreadRdy; 		// ready flag
} ThreadResource;


/* prototypes */
 void* Timer_Irq_Thread(void* resource);
 void getStates();
 double cascade(double xin, struct biquad *fa, int ns, double ymin, double ymax);
 void bws_mode();
 void fall_protect_mode();

/* definitions */
#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x))

/*global declaration*/
 NiFpga_Session myrio_session;

static State_Type curr_state = FALL;
static double *mode;
static double *bws;
static double *VDAout;
static double *omega;
static double *force;
static double *vel;
static double *pos;

static double vel_threshold = -.5;
static double pos_threshold = .5;

static MyRio_Encoder encC0;
static MyRio_Aio AIC0;
static MyRio_Aio AOC0;

//import FSM function headers here
#include "float.h"
#include "fallProtection.h"
#include "BWS.h"

static void (*state_table[]) (void) = {fall_protect_mode, bws_mode, float_mode};

int main(int argc, char **argv)
{
	NiFpga_Status status;

    status = MyRio_Open();		    			// open FPGA session
    if (MyRio_IsNotSuccess(status)) return status;

    //Set up table
    char* tableName = "Lift Control";
    int tab_len = 7;
    table tabs[] = {
			{"Mode        ", 1, curr_state},
			{"BWS           ", 1, .5},
			{"VDAout    (V)", 0, 0.0},
			{"Omega (rad/s)", 0, 0},
			{"Force     (N)", 0, 0},
			{"Vel     (m/s)", 0, 0},
			{"Pos       (m)", 0, 0}
    };

    //Set up threads
	uint32_t stat1;
	MyRio_IrqTimer irqTimer0;
	ThreadResource irqThread0;
	pthread_t thread;

	// Initialize Interrupt requests and thread.
	irqTimer0.timerWrite = IRQTIMERWRITE;
	irqTimer0.timerSet = IRQTIMERSETTIME;

	stat1 =Irq_RegisterTimerIrq(&irqTimer0, &irqThread0.irqContext, timeoutValue);
	if (stat1) return stat1;

	irqThread0.irqThreadRdy = NiFpga_True;
	irqThread0.a_table = tabs;

	//Create thread
	stat1 = pthread_create(&thread,NULL,Timer_Irq_Thread,&irqThread0);

	//Create ctable
	ctable2(tableName, tabs, tab_len);

	irqThread0.irqThreadRdy = NiFpga_False;
	stat1 = pthread_join(thread, NULL);

	status = Irq_UnregisterTimerIrq(&irqTimer0, irqThread0.irqContext);
	status = MyRio_Close();

	return status;
}

void* Timer_Irq_Thread(void* resource) {
	ThreadResource* threadResource = (ThreadResource *) resource;
	uint32_t irqAssert;

	mode = &((threadResource->a_table+0)->value);
	bws = &((threadResource->a_table+1)->value);
	VDAout = &((threadResource->a_table+2)->value);
	omega = &((threadResource->a_table+3)->value);
	force = &((threadResource->a_table+4)->value);
	vel = &((threadResource->a_table+5)->value);
	pos = &((threadResource->a_table+6)->value);

	//setting up channels
	EncoderC_initialize(myrio_session, &encC0);
	Aio_InitCI0(&AIC0); 			// Initialize input 0
	Aio_InitCO0(&AOC0); 		// Initialize output 0
	Aio_Write(&AOC0, 0);

	while(threadResource->irqThreadRdy == NiFpga_True) {
		//Waiting for interrupt and scheduling next interrupt.
		Irq_Wait( threadResource->irqContext,TIMERIRQNO,&irqAssert,(NiFpga_Bool*) &(threadResource->irqThreadRdy));

		NiFpga_WriteU32( myrio_session,IRQTIMERWRITE,	timeoutValue);
		NiFpga_WriteBool( myrio_session,	IRQTIMERSETTIME,NiFpga_True);

		if (irqAssert){
			//measure states
			getStates();

			//FSM
			state_table[curr_state]();

			Aio_Write(&AOC0,*VDAout);

			if (*vel>5){
				threadResource->irqThreadRdy = NiFpga_False;
				Aio_Write(&AOC0,0);
				putchar_lcd(220);
				printf("\fEmergency Stop: Velocity Threshold Exceeded\n");
			}else if (*pos>1){
				threadResource->irqThreadRdy = NiFpga_False;
				Aio_Write(&AOC0,0);
				putchar_lcd(220);
				printf("\fEmergency Stop: Position Threshold Exceeded\n");
			}

			Irq_Acknowledge(irqAssert);
		}

	}

	Aio_Write(&AOC0,0);

	pthread_exit(NULL);
	return NULL;
}


/*------------------------------------------------------------------------------------------------
 * Function  getStates()
 * Purpose:
 * Arguments:
 * Returns:
 ------------------------------------------------------------------------------------------------*/
void getStates(){
	static int first = 1;												// flag for first time running

	int count;
	static int count_ini;
	static int count_prev;
	double angle;
	double ang_vel;

	double VADin;
	double A = -0.003856393532029;
	double B = 0.027032187011281;
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

	angle = 2*M_PI*(count-count_ini)/BDI/N;
	ang_vel = 2*M_PI*(count-count_prev)/BDI/N/(timeoutValue*1e-6);
	count_prev = count;

	v_spr = (x_spr-x_spr_prev)/(timeoutValue*1e-6);
	x_spr_prev = x_spr;

	*omega = ang_vel;
	*force = k*x_spr;
	*pos = l0+r*angle-2*x_spr;
	*vel =  r*ang_vel-2*v_spr;
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

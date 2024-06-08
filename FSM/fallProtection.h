#ifndef FALLPROTECT_H
#define FALLPROTECT_H

#include "T1.h"
#include "myLQR1kgc.h"
#include "myLPF.h"

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
	double	omega;
	double	force;
	double	vel;
	double	pos;
} SysStates;

void fall_protect_mode();
void sramp2(Param param, MStates *ref_state, double bti, uint32_t *itime);
double lqr(SysStates states,SysStates ref, double K[], double min, double max);

void fall_protect_mode(){
	static int initialized = 0;

	//set up path profile parameters
	static Param param;
	param.vmax = .3;									// maximum velocity (m/s)
	param.amax = 5;									// maximum acceleration (m/s^2)
	param.xf = .5;										// final position (m)

	//set up states
	SysStates ref;
	SysStates states;

	//set up s-ramp
	MStates mass_ref;
	double bti = timeoutValue/1e6;
	static uint32_t itime = 0;												// current time index

	//set up controller commands
	double torque;

	if (!initialized){
			param.vi= *vel;									// initial velocity (m/s)
			param.xi = *pos;								// initial position (m)
			itime = 0;

			initialized = 1;
	}

	//format current state
	states.omega = *omega;
	states.force = *force;
	states.vel =  cascade(*vel, LPF_FP,LPF_FP_ns, -INFINITY, INFINITY);
	states.pos = *pos;

	//get reference curve
	sramp2(param,&mass_ref,bti,&itime);

	//get reference state
	ref.omega = mass_ref.vel/r;
	ref.force = mass_ref.acc*2*m+2*m*g;
	ref.vel = mass_ref.vel;
	ref.pos = mass_ref.pos;

	// LQR controller
	torque = lqr(states,ref,K,-tmax,tmax);
	*VDAout = torque/N/km/ka;

	curr_state = (State_Type) *mode;
	if (curr_state != FALL){
		initialized = 0;
	}
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

#endif

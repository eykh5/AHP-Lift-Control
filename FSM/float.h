#ifndef FLOAT_H
#define FLOAT_H

#include "T1.h"
#include "PIDF0.h"
#include "PD.h"
#include "lowpass_45hz.h"
#include "lowpass_pot.h"

//prototype
void float_mode();

/*-------------------------------------------------------------------------------------------------------------------------------------
Function FLOAT

	Purpose:			The timer interrupt. The function initially initializes the analog I/O channels. Subsequently, it communicates
							with the main thread via the threadResource struct pointer and asserts IRQ every 5 ms.
							Updates the biquad structure as well as the table values
							It performing biquad cascade of the error signal and conducts DAC of the output signal.
							Reference velocity, its previous reference speed, and torque values are stored in a buffer to later save as .mat file.
	Parameters: 	None
	Returns:  		None
------ --------------------------------------------------------------------------------------------------------------------------------*/
void float_mode(){
	double GAIN1 = 0.2;
	double GAIN2 = 1.0;

	double vref_act, pos_act, vel_act;
	double enc_vel, enc_pos, pot_vel, pot_pos, x_ref, Pm;
	double W = m*g;

	static int F_init = 0;
	static double T_initial;
	double Vm, ref_mass, PD_output, error1, error2, Fk, output, lowpass_output, lowpass_Vm;

	Vm = *vel;
	Fk = *force;
	Pm = *pos;
	vel_act = *omega;
	pot_vel = -0.5*(Vm - 0.05*vel_act);
	lowpass_Vm = Vm;

	pot_vel = cascade(pot_vel, LPF_POT, 1, -7.5, 7.5);

	// Defining Initial Tension to overcome hysteresis
	if (F_init == 0) {
		T_initial = Fk/2;
		F_init++;
		PIDF0[0].y1 = *VDAout;
		PIDF0[0].y2 = *VDAout;
	}

	ref_mass = GAIN1*(W-Fk/2);

	if (fabs(T_initial - Fk/2) < 0.75) {
		ref_mass = 0;
	}

	error1 = ref_mass - GAIN2*lowpass_Vm;

	PD_output = cascade(error1, PD1, PIDF1_ns, -7.5, 7.5);
	error2 = PD_output - vel_act;
	output = cascade(error2, PIDF0,  PIDF0_ns, -7.5, 7.5);
	lowpass_output =  cascade(output, LPF, 1, -7.5, 7.5);
	*VDAout = lowpass_output;

	/*
	if (*vel < vel_threshold && *pos < pos_threshold){
		 *mode = 0;
	}
	*/
	curr_state = (State_Type) *mode;
}

#endif

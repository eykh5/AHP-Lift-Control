#ifndef BWS_H
#define BWS_H


/* includes */
#include "T1.h"
#include "PIDF_BWS.h"
#include "FILTER_BWS.h"
#include "SWITCH_FILT.h"

/* prototypes */

void bws_mode();
void init();

//Don't need to include an exist state because ctable takes care of it.
// #include "lowpass_pot.h"


void bws_mode() {
	//Calculating position and voltage to send to motor.
	static int first = 0;

	if (first == 0) {
		init();
		first++;
	}

	double weight = m * g;
	double bws_amount = weight * (*bws);
	bws_amount = cascade(bws_amount, SWITCH_FILT, SWITCH_FILT_ns, 0, 25);
	double er = -*force/2 + bws_amount;
	double signal = cascade(er, PIDF_BWS, PIDF_BWS_ns, -7.5, 7.5);
	signal = cascade(signal, FILTER_BWS, FILTER_BWS_ns, -7.5, 7.5);
	*VDAout = signal;



	if (*vel < vel_threshold && *pos < pos_threshold){
		 *mode = 0;
}


curr_state = (State_Type) *mode;
}

void init() {
	SWITCH_FILT[0].y1 = *bws;
	SWITCH_FILT[0].y2 = *bws;
	PIDF_BWS[0].y1 = *VDAout;
	PIDF_BWS[0].y2 = *VDAout;
}

#endif

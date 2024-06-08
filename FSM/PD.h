//---Transfer function: Torque to motor velocity
//---15-May-2024 18:09:19
    int         PIDF1_ns = 1;              // number of sections
    static	struct	biquad PD1[]={   // define the array of floating point biquads
        {1.961082e+03, -1.516461e+03, 0.000000e+00, 1.000000e+00, -1.834320e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };

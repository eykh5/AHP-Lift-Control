//---FILTER For BWS
//---23-May-2024 09:31:02
    int         SWITCH_FILT_ns = 1;              // number of sections
    static	struct	biquad SWITCH_FILT[]={   // define the array of floating point biquads
        {4.761905e-02, 4.761905e-02, 0.000000e+00, 1.000000e+00, -9.047619e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };

//---Transfer function: Low-pass for Vmass
//---13-May-2024 22:05:21
    int         LPF_ns = 1;              // number of sections
    static	struct	biquad LPF[]={   // define the array of floating point biquads
        {1.011236e-01, 1.011236e-01, 0.000000e+00, 1.000000e+00, -7.977528e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };

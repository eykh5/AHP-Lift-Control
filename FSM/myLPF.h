//---Fall Protection Velocity Filter
//---27-May-2024 17:31:00
//    char        headerTime[] = "27-May-2024 17:31:00";
    int         LPF_FP_ns = 1;              // number of sections
//    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad LPF_FP[]={   // define the array of floating point biquads
        {1.357552e-01, 1.357552e-01, 0.000000e+00, 1.000000e+00, -7.284895e-01, 0.000000e+00, 0, 0, 0, 0, 0}
        };

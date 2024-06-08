//---Transfer function: Torque to motor velocity
//---30-Apr-2024 18:37:18
    char        headerTime[] = "30-Apr-2024 18:37:18";
    int         T0_ns = 1;              // number of sections
    uint32_t    timeoutValue = 5000;      // time interval - us; f_s = 200 Hz
    static	struct	biquad T0[]={   // define the array of floating point biquads
        {2.975423e-01, -1.806835e-01, 1.325612e-02, 1.000000e+00, -4.656099e-01, -5.343901e-01, 0, 0, 0, 0, 0}
        };

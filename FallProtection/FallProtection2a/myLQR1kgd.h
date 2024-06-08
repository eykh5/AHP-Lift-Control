//---LQR fall protection
//---27-May-2024 21:23:55
    char        headerTime[] = "27-May-2024 21:23:55";
    uint32_t    timeoutValue = 5000;      // time interval (us): f_s = 200 Hz
    static	double K[] = {4.167409e-03,-1.339240e-03,1.575270e+01,2.646953e+01,};
    static	double r = 5.000000e-02;
    static	double J = 2.500000e-04;
    static	double k = 690;
    static	double m = 1;

//---LQR fall protection
//---27-May-2024 21:10:50
    char        headerTime[] = "27-May-2024 21:10:50";
    uint32_t    timeoutValue = 5000;      // time interval (us): f_s = 200 Hz
    static	double K[] = {3.772184e-02,1.618114e-01,4.097082e+00,3.108270e+01,};
    static	double r = 5.000000e-02;
    static	double J = 2.500000e-04;
    static	double k = 690;
    static	double m = 1;

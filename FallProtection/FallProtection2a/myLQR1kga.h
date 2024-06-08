//---LQR fall protection
//---23-May-2024 13:10:27
    char        headerTime[] = "23-May-2024 13:10:27";
    uint32_t    timeoutValue = 5000;      // time interval (us): f_s = 200 Hz
    static	double K[] = {2.531927e-02,7.099087e-02,1.918291e+00,1.492285e+01,};
    static	double r = 5.000000e-02;
    static	double J = 2.500000e-04;
    static	double k = 690;
    static	double m = 1;

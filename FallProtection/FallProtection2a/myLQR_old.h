//---LQR fall protection
//---08-May-2024 03:19:14
    char        headerTime[] = "08-May-2024 03:19:14";
    uint32_t    timeoutValue = 5000;      // time interval (us): f_s = 200 Hz
    static	double K[] = {7.473746e-02,6.238657e-01,1.592808e+01,1.165072e+02,};
    static	double r = 5.000000e-02;
    static	double J = 2.500000e-04;
    static	double k = 690;
    static	double m = 1;

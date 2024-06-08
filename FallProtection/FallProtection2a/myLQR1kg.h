//---LQR fall protection
//---23-May-2024 12:32:46
    char        headerTime[] = "23-May-2024 12:32:46";
    uint32_t    timeoutValue = 5000;      // time interval (us): f_s = 200 Hz
    static	double K[] = {1.658467e-02,2.923026e-02,9.731719e-01,7.503374e+00,};
    static	double r = 5.000000e-02;
    static	double J = 2.500000e-04;
    static	double k = 690;
    static	double m = 1;

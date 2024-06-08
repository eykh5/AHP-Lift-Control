//---LQR fall protection
//---16-May-2024 18:44:31
//    char        headerTime[] = "16-May-2024 18:44:31";
    uint32_t    timeoutValue = 5000;   		// time interval (us): f_s = 200 Hz
    static	double r = 5.000000e-02;			// spool radius (m)
    static	double J = 2.500000e-04;			// spool moment of inertia (kg*m^2)
    static	double k = 690.;							// spring stiffness	(N/m)
    static	double m = 1.;		// mass (kg)
	static double BDI = 2000	;					// encoder counts per revolution, motor side
	static double N = 5.9;							// gear ratio
	static double km	 = .0507;					// motor gain
	static double ka  = .41;						// amp gain
	static double tmax = 1;						// max torque
	static double l0 = 0;								// position offset
	static double g = 9.81;							// gravity

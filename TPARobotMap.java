package org.usfirst.frc.team3944.robot;

public class TPARobotMap {

	//TPA Joystick
	public static final int JoystickPort = 0;
			 
	// CAN Talon Device IDs   
	public static final int frontLeftCAN_ID 	= 1;
	public static final int backLeftCAN_ID 		= 2;
	public static final int frontRightCAN_ID 	= 3;
	public static final int backRightCAN_ID 	= 4;
	
	public static final int masterShooter_ID	= 20;
	public static final int slaveShooter_ID		= 21;
	public static final int loader_ID			= 22;
	public static final int lifter_ID			= 23;
	
	
	// Permanent Button Assignment
	        public static final int autoShooterButton	   = 1;
		 	public static final int liftAutoButton         = 2;
	        public static final int liftUpButton           = 3;
	        public static final int zeroOutButton          = 4;
	        public static final int liftDownButton         = 5;
	        public static final int resetNavButton         = 6;
			public static final int turnLeftButton         = 7;
			public static final int turnRightButton        = 8;
			public static final int autoLiftClick     	   = 9;
			public static final int manualShooterButton    = 10;
			public static final int turnOnLightButton      = 11;
			public static final int manualLoaderButton 	   = 12;
			
			
	// Limit Switches 
			public static final int loaderLimitSwitch_ID	= 0;
			public static final int lifterFrontLimitSwitch  = 4;
			public static final int lifterBackLimitSwitch   = 1;
    // Speeds and tuning parameters 
			public static final double loaderMotorSpeed		= 1.0;
			//public static final double shooterOptimalSpeed 	= 5800.0; // This is the tuned optimal shooting speedCOMPETITION
			public static final double shooterOptimalSpeed  = 5300.0;
			public static final double shooterOptimalSpeed2  = 5600.0;
			

}

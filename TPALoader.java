package org.usfirst.frc.team3944.robot;
/*
 * TPALoader is used to load the ball into the nest. Waits till shooter wheels reach 
 * maximum velocity, then assists the ball into the shooter wheels
 */
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TPALoader {
    // Declare new loadMotor CANTalon object
	public static CANTalon loaderMotor = new CANTalon(TPARobotMap.loader_ID);
	
	private TPAJoystick joystick;
	private DigitalInput loaderLimitSwitch;
	
	// Constructor
	public TPALoader(TPAJoystick joystick, DigitalInput aswitch){	
		this.joystick = joystick;
		loaderLimitSwitch = aswitch;
	}
	
	// Loads ball manually via sustained button push
	// False = Limit switch is engaged, true = Limit switch is not engaged (we think)
	public void loadBallManual(double speed){
		if(loaderLimitSwitch.get() == true) {
		
		 // Invert for proper motor direction 	
		 //loaderMotor.setInverted(true); COMPETITION BOT
		 loaderMotor.setInverted(false);
		 loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
		 loaderMotor.set(speed);
		
		} 
		//  This condition has as dependency on the autoShooterButton not being pressed. Limit switch False kills the loader motor.
		//  TPARobotMap.autoShooterButton must be pressed in order for shooter to fire. 
		
		if(joystick.getRawButton(TPARobotMap.autoShooterButton) == false && loaderLimitSwitch.get() == false) {
			 // Invert for proper motor direction 	
			 loaderMotor.setInverted(true);
			 loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
			 loaderMotor.set(0);
		}  
	}
	
	// Loads ball automatically into the shooter wheels via velocity signal from TPAShooter class
		public void loadBallAuto(double optimalSpeed){
			// 1440 Encoder Edge events equates to the amount of power going to the motor
			// value of 1440 = max power 
			SmartDashboard.putString("Get Shooter Speed from loadBallAuto: ", "" +TPAShooter.masterShooter.getSpeed());
			
			if(TPAShooter.masterShooter.getSpeed() >= optimalSpeed /*&& loaderLimitSwitch.get() == false*/ ) {
			 // Invert for proper motor direction 	
			 //loaderMotor.setInverted(true); COMPETITION>????
			 loaderMotor.setInverted(false);
			 loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
			 loaderMotor.set(1.0);
			 //Timer.delay(1);
			 
			}
			//after the shooter shoots he will have his finger on the button and we want it to stop
		/*	if(TPAShooter.masterShooter.getSpeed() >= optimalSpeed && loaderLimitSwitch.get() == true) {
				 // Invert for proper motor direction 	
				 loaderMotor.setInverted(true);
				 loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
				 loaderMotor.set(0.0);
			} */
			 
		}

}

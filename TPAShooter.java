package org.usfirst.frc.team3944.robot;

/*
 * TPAShooter Shoots the ball when manually engaged sends signal to loader to assist ball
 * into the shooting wheels
 */

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;



public class TPAShooter {
	
	public static CANTalon masterShooter = new CANTalon(TPARobotMap.masterShooter_ID);
	public static CANTalon slaveShooter = new CANTalon(TPARobotMap.slaveShooter_ID);
	private TPAJoystick joystick;
	private DigitalInput nestSwitch;
	private boolean resetEnc;
	
	public TPAShooter(TPAJoystick joystick, DigitalInput aSwitch){
		this.joystick = joystick;
		nestSwitch = aSwitch;
		// Sets the correct motor rotation for the these motors 
		masterShooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		slaveShooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		masterShooter.setInverted(true);
		masterShooter.changeControlMode(TalonControlMode.PercentVbus);
		slaveShooter.changeControlMode(TalonControlMode.Follower);
		//slaveShooter.reverseOutput(true);//COMPETITION
		slaveShooter.reverseOutput(true);
		resetEnc = false;
	}

	// Sustained button push, fires up the wheels 
	public void shootBallManual(double speed) {
		SmartDashboard.putString("Digital Switch value: ", "" +nestSwitch.get());
		   
		if(joystick.getRawButton(TPARobotMap.manualShooterButton)== true && nestSwitch.get() == true) {			
			masterShooter.set(speed);
			// always set follower to master device ID
			slaveShooter.set(masterShooter.getDeviceID());
		}
	 	if(joystick.getRawButton(TPARobotMap.manualShooterButton)== false && nestSwitch.get() == true) {
			masterShooter.set(0.0);
			// always set follower to master device ID
			slaveShooter.set(masterShooter.getDeviceID());
		}  
	}
	
	// This automatically starts the shooting cycle with a button push. 
	public void shootBallAuto(double speed){
		/*if(joystick.getRawButton(TPARobotMap.autoShooterButton) == true && resetEnc == false){
			TPARobotDrive.frontLeftMotor.setPosition(0);
			resetEnc = true;
		}*/
		if(joystick.getRawButton(TPARobotMap.autoShooterButton) == true /*&& nestSwitch.get() == false*/){
			masterShooter.set(speed);
			slaveShooter.set(masterShooter.getDeviceID());
			SmartDashboard.putString("masterShooter.getSpeed(): ", ""+masterShooter.getSpeed());
			SmartDashboard.putString("shootingencoderposition: ", ""+TPARobotDrive.frontLeftMotor.getPosition());
		}/*if(joystick.getRawButton(TPARobotMap.autoShooterButton) == false){
			resetEnc = false;
		}*/
		// this logic in lines 57 thru 60 is causing iterative choking
/*		if(joystick.getRawButton(TPARobotMap.ShooterButton)== false && nestSwitch.get() == true) {
			masterShooter.set(0.0);
			slaveShooter.set(masterShooter.getDeviceID());
		} */
	}

}

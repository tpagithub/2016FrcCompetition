package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class TPALifter {
	public static CANTalon lifterMotor = new CANTalon(TPARobotMap.lifter_ID);
	//class used for interacting with obstacles in teleop and auto
	private TPAJoystick joystick;
	private DigitalInput extendedSwitch, retractedSwitch;
	private boolean autoForward, autoClick, joystickClick;
	public TPALifter(TPAJoystick joystick, DigitalInput extended, DigitalInput retracted){
		this.joystick = joystick;
		extendedSwitch = extended;
		retractedSwitch = retracted;
		autoForward = true;
		autoClick = false;
		joystickClick = false;
	}
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//LIMIT SWITCHS ARE REVERSE THEIR NORMAL STATE: FALSE=HIT AND TRUE=NOTHIT,NOW THEY ARE TRUE=HIT AND FALSE=NOTHIT!!!!!!!!
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	public void runLifterDownManual(){
		if(joystick.getRawButton(TPARobotMap.liftDownButton) == true && extendedSwitch.get() == false ||
				joystick.getRawButton(TPARobotMap.liftDownButton) == true && extendedSwitch.get() == false && retractedSwitch.get() == true){
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			//lifterMotor.set(1.0);//COMPETITION
			lifterMotor.set(-1.0);
			
		}if(joystick.getRawButton(TPARobotMap.liftDownButton) == false && joystick.getRawButton(TPARobotMap.liftUpButton) == false
				|| extendedSwitch.get() == true && joystick.getRawButton(TPARobotMap.liftUpButton) == false){
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			lifterMotor.set(0);
		}
	}
	
	public void runLifterUpManual(){
		if(joystick.getRawButton(TPARobotMap.liftUpButton) == true && retractedSwitch.get() == false ||
				joystick.getRawButton(TPARobotMap.liftUpButton) == true && retractedSwitch.get() == false && extendedSwitch.get() == true){
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			//lifterMotor.set(-1.0);//COMPETITION
			lifterMotor.set(1.0);
		}
		if(joystick.getRawButton(TPARobotMap.liftUpButton) == false  && joystick.getRawButton(TPARobotMap.liftDownButton) == false
				|| retractedSwitch.get() == true && joystick.getRawButton(TPARobotMap.liftDownButton) == false){
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			lifterMotor.set(0);
		}
	}
	
	public void autoLiftSustained() {
		if (joystick.getRawButton(TPARobotMap.liftDownButton) == false && joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == true 
				&& extendedSwitch.get() == false && autoForward == true|| joystick.getRawButton(TPARobotMap.liftDownButton) == false 
				&& joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == false 
				&& extendedSwitch.get() == false && autoForward == true) {
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			lifterMotor.set(-1.0);
			//lifterMotor.set(1.0);
		}
		if (joystick.getRawButton(TPARobotMap.liftDownButton) == false && joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == false 
				&& extendedSwitch.get() == true && autoForward == true) {
			Timer.delay(0.5);
			autoForward = false;
		}
		if (joystick.getRawButton(TPARobotMap.liftDownButton) == false && joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == false 
				&& extendedSwitch.get() == true && autoForward == false|| joystick.getRawButton(TPARobotMap.liftDownButton) == false 
				&& joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == false 
				&& extendedSwitch.get() == false && autoForward == false) {
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			lifterMotor.set(1.0);
			//lifterMotor.set(-1.0);
		}
		if (joystick.getRawButton(TPARobotMap.liftDownButton) == false && joystick.getRawButton(TPARobotMap.liftUpButton)== false
				&& joystick.getRawButton(TPARobotMap.liftAutoButton) == true && retractedSwitch.get() == true 
				&& extendedSwitch.get() == false && autoForward == false) {
			 
			lifterMotor.changeControlMode(TalonControlMode.PercentVbus);
			lifterMotor.set(0.0);
			Timer.delay(1);
			autoForward = true;
		}
	}
	
	public void autoLiftClick(){
		if(joystick.getRawButton(TPARobotMap.autoLiftClick) == true && autoClick == false && joystickClick == false){
			autoClick = true;	
			joystickClick = true;
		}if(autoClick == true && extendedSwitch.get() == false && joystickClick == true){
			lifterMotor.set(-1.0);
			//lifterMotor.set(1.0);
		}if(autoClick == true && extendedSwitch.get() == true && joystickClick == true){
			Timer.delay(.5);
			autoClick = false;
		}if(autoClick == false && joystickClick == true){
			lifterMotor.set(1.0);
			//lifterMotor.set(-1.0);
		}if(autoClick == false && retractedSwitch.get() == true && joystickClick == true){
			lifterMotor.set(0.0);
			joystickClick = false;
		}
	}

}


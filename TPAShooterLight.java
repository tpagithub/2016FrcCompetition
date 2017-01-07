package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;

public class TPAShooterLight extends Relay{
	
	private TPAJoystick joystick;
	private boolean isOn;
	
	public TPAShooterLight(int port, TPAJoystick joystick){
		super(port);
		this.joystick = joystick;
	}
	
	public void turnOnLight(){
		if(joystick.getButtonPush(TPARobotMap.turnOnLightButton) == true && isOn == false && Robot.loaderSwitch.get() == false){
			isOn = true;
		}
		if(isOn == true && Robot.loaderSwitch.get() == false){
			setDirection(Relay.Direction.kForward);
			set(Relay.Value.kOn);
		}
		if(TPAShooter.masterShooter.getSpeed() < 2 && isOn == true && Robot.loaderSwitch.get() == false){
			DriverStation.reportError("THE LIGHT IS ON AND YOU HAVE NOT SHOT YET!", true);
		}
		if(TPAShooter.masterShooter.getSpeed() >= TPARobotMap.shooterOptimalSpeed && isOn == true && Robot.loaderSwitch.get() == false){
			set(Relay.Value.kOff);
			isOn = false;
		}
	}

}

package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.AnalogInput;

// Extends AnalogInput class to support analog switch used in autonomous
public class TPAAnalogInput extends AnalogInput {
	
	public TPAAnalogInput(int channel){
		super(channel);
	}

}

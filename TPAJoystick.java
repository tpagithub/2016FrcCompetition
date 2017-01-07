package org.usfirst.frc.team3944.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*
 * TPAJoystick extends Joystick. It is iterating constantly while the robot is
 * running in Telelo. The buttonState is constantly being updated when 
 * getButtonPush is called in other classes.
 * ONCE TELEOP IS ENABLED THIS SETS EVERY ELEMENT IN THE BUTTONSTATE BOOLEAN ARRAY TO TRUE
 * THEN IMMEDIATELY WHEN NO BUTTON IS PUSED IT CHANGES IT TO FALSE
 */
public class TPAJoystick extends Joystick {
	// Declare Boolean for the button states
	private boolean[] buttonState = new boolean[12];
	private int counter;
	private boolean returnGarageValue;
	// Constructor: initialize the boolean array  
	// The boolean exists to shield button pushes from constant iterations
	public TPAJoystick(final int aPort) {
        super(aPort);
        for(int i=0; i<buttonState.length; i++) { 
            buttonState[i] = true;
        }
        counter = 0;
    }
	
	// Button is pushed and is passed the button number
	public boolean getButtonPush(int button) {
        boolean returnValue;
        // Call getRawButton from the parent Joystick class to acquire boolean value for button. 
        // Sets newState boolean variable, unpressed state is false, pressed state is true
        boolean newState = super.getRawButton(button);
        
        // newState is true and newState for the button being pushed is not already equal to true
        // because that would mean it is sustained push
        if(newState == true && newState != buttonState[button - 1]) {
        	//SmartDashboard.putString("Button Value for newState inside of if statement: ", "" +newState);
        	
        	// returns true to a calling object
                returnValue = true;
              //  SmartDashboard.putString("the return value inside if loop is: ", "" +returnValue);
        }
        //return false to calling object
        else {
        	returnValue = false;
        	//SmartDashboard.putString("the return value inside if loop else statement is: ", "" +returnValue);
        }
        
       // SmartDashboard.putString("Button Value for newState outside of if loop: ", "" +newState);
       // SmartDashboard.putString("the return value outside if loop is: ", "" +returnValue);
        // reset button state 
        buttonState[button - 1] = newState;
        return(returnValue);
    } // End getButtonPush
	
	public boolean garageButton(int button) {
		
		boolean newState = super.getRawButton(button);
		
		if(newState == true ) {
			counter++;
			 
			returnGarageValue = true;
		}
		
		if (counter == 2) {
			counter = 0;
			returnGarageValue = false;
		}
      return(returnGarageValue);
		
	}

}

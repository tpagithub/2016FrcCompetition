package org.usfirst.frc.team3944.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TPAAutoTasks implements PIDOutput {
	
    
  
    private int taskCounterA;
    private int taskCounterB;
    
	private PIDController pid;
	private static double pidOutput;
	private AHRS ahrs;
	
    public TPAAutoTasks(AHRS ahrs) {
    	this.ahrs = ahrs; 
        pid = new PIDController(0.03, 0.1, 0.1, 0.01, ahrs, this);
        pid.setInputRange(-180.0f,  180.0f);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsoluteTolerance(1.0f);
        pid.setContinuous(true);	
        ahrs.reset();
        taskCounterA = 0;
        taskCounterB = 0;
        TPARobotDrive.frontLeftMotor.setPosition(0); 
    }

    
  public void moat() {
	  SmartDashboard.putString("getPosition Before First If: ", "" + TPARobotDrive.frontLeftMotor.getPosition());
	  
	 if(Math.abs( TPARobotDrive.frontLeftMotor.getPosition()) < calculateDistance(6) && (taskCounterA == 0) && (taskCounterB == 0)){
	    
		 
		  TPARobotDrive.frontLeftMotor.setF(0.5); //.99
		  TPARobotDrive.frontLeftMotor.setP(0.04); //.04
		  TPARobotDrive.frontLeftMotor.setI(0.0); //0
		  TPARobotDrive.frontLeftMotor.setD(0.0); //0
		     
		  TPARobotDrive.frontRightMotor.setF(0.5); //.99
		  TPARobotDrive.frontRightMotor.setP(0.04); //.04
		  TPARobotDrive.frontRightMotor.setI(0.0); //0
		  TPARobotDrive.frontRightMotor.setD(0.0); //0
	    
		     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
		     TPARobotDrive.frontLeftMotor.set(300); //1032 max speed if F is .99 and P .04
		     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
		    
		     TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
		     TPARobotDrive.frontRightMotor.set(300); //1032 max speed if F is .99 and P .04
		     TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());	  
		   


		   SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
		   SmartDashboard.putString("taskCounterB: ", "" + taskCounterB); 
	       SmartDashboard.putString("distance in the  first loop: ", "" + calculateDistance(15.0));
		   SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
		   SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
           SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
           SmartDashboard.putString("talon.getPosition Joe", "" + TPARobotDrive.frontLeftMotor.getPosition());
           SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
           SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
           SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
           SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
     	   SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
	       SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());
	     
           
	   }
	  
	   if (Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >= calculateDistance(6) && (taskCounterA == 0) && (taskCounterB == 0))  {
			   
		   TPARobotDrive.frontLeftMotor.setPosition(0);
		   TPARobotDrive.frontRightMotor.setPosition(0);
		    taskCounterA = 1;
		    taskCounterB = 1;
		    Timer.delay(0.1);
		    SmartDashboard.putString("distance in the second loop: ", "" + calculateDistance(15.0));
		    SmartDashboard.putString("get zero enc pos FL: ", "" +TPARobotDrive.frontLeftMotor.getPosition());
		    SmartDashboard.putString("get zero enc pos FR: ", "" +TPARobotDrive.frontRightMotor.getPosition());
		 
	  }
	  
	 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(1.5)) && (taskCounterA == 1) && (taskCounterB == 1)){
		 //talonBR.reverseOutput(true);
		 //talonFL.reverseOutput(true);
		 //talonFR.reverseOutput(true);
		 TPARobotDrive.frontLeftMotor.setF(0.5); //.99
		 TPARobotDrive.frontLeftMotor.setP(0.04); //.04
		 TPARobotDrive.frontLeftMotor.setI(0.0); //0
		 TPARobotDrive.frontLeftMotor.setD(0.0); //0
		     
		 TPARobotDrive.frontRightMotor.setF(0.5); //.99
		 TPARobotDrive.frontRightMotor.setP(0.04); //.04
		 TPARobotDrive.frontRightMotor.setI(0.0); //0
		 TPARobotDrive.frontRightMotor.setD(0.0); //0
		 
		     //200, and -200 with a 5.1 encoder value
		 
		     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
		     TPARobotDrive.frontLeftMotor.set(200); //1032 max speed if F is .99 andP .04
		     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
	         
	         TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
	         TPARobotDrive.frontRightMotor.set(-200); //1032 max speed if F is .99 andP .04
	         TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());
	       
	    
	    SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
	    SmartDashboard.putString("taskCounterB: ", "" + taskCounterB);
	    SmartDashboard.putString("distance: ", "" + calculateDistance(15));
		SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
		SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
        SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
        SmartDashboard.putString("talon.getPosition", "" + TPARobotDrive.frontLeftMotor.getPosition());
        SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
        SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
        SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
        SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
  	    SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
	    SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

	 }
	 
	 
	   if (Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >= calculateDistance(1.5) && (taskCounterA == 1) && (taskCounterB == 1))  {
		   
		   TPARobotDrive.frontLeftMotor.setPosition(0);
		   TPARobotDrive.frontRightMotor.setPosition(0);
		    taskCounterA = 2;
		    taskCounterB = 2;
		    Timer.delay(0.1);
		    SmartDashboard.putString("distance in the second loop: ", "" + calculateDistance(15.0));
		    SmartDashboard.putString("get zero enc pos FL: ", "" +TPARobotDrive.frontLeftMotor.getPosition());
		    SmartDashboard.putString("get zero enc pos FR: ", "" +TPARobotDrive.frontRightMotor.getPosition());
		 
	  }
	 
	 
	 
	
	 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(5.0)) && (taskCounterA == 2) && (taskCounterB == 2)){
		 //talonBR.reverseOutput(true);
		 //talonFL.reverseOutput(true);
		 //talonFR.reverseOutput(true);
		 TPARobotDrive.frontLeftMotor.setF(0.5); //.99
		 TPARobotDrive.frontLeftMotor.setP(0.04); //.04
		 TPARobotDrive.frontLeftMotor.setI(0.0); //0
		 TPARobotDrive.frontLeftMotor.setD(0.0); //0
		     
		 TPARobotDrive.frontRightMotor.setF(0.5); //.99
		 TPARobotDrive.frontRightMotor.setP(0.04); //.04
		 TPARobotDrive.frontRightMotor.setI(0.0); //0
		 TPARobotDrive.frontRightMotor.setD(0.0); //0
		     
		     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
		     TPARobotDrive.frontLeftMotor.set(225); //1032 max speed if F is .99 andP .04
		     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
	         
	         TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
	         TPARobotDrive.frontRightMotor.set(225); //1032 max speed if F is .99 andP .04
	         TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());
	       
	    
	    SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
	    SmartDashboard.putString("taskCounterB: ", "" + taskCounterB);
	    SmartDashboard.putString("distance: ", "" + calculateDistance(15));
		SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
		SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
        SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
        SmartDashboard.putString("talon.getPosition", "" + TPARobotDrive.frontLeftMotor.getPosition());
        SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
        SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
        SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
        SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
  	    SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
	    SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

	 }
	 
	 if (Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >= calculateDistance(5.0) && (taskCounterA == 1) && (taskCounterB == 2))  {
		   
		   TPARobotDrive.frontLeftMotor.setPosition(0);
		   TPARobotDrive.frontRightMotor.setPosition(0);
		   TPARobotDrive.frontLeftMotor.set(0);
		   TPARobotDrive.backLeftMotor.set(0);
		    taskCounterA = 3;
		    taskCounterB = 3;
		    Timer.delay(0.1);
		    SmartDashboard.putString("distance in the second loop: ", "" + calculateDistance(15.0));
		    SmartDashboard.putString("get zero enc pos FL: ", "" +TPARobotDrive.frontLeftMotor.getPosition());
		    SmartDashboard.putString("get zero enc pos FR: ", "" +TPARobotDrive.frontRightMotor.getPosition());
		 
	  }
	
 	 
  }  // End Moat
  
  public void drawBridge() {
	  SmartDashboard.putString("getPosition Before First If: ", "" + TPARobotDrive.frontLeftMotor.getPosition());
	  
		 if(Math.abs( TPARobotDrive.frontLeftMotor.getPosition()) < calculateDistance(15.0) && (taskCounterA == 0) && (taskCounterB == 0)){
		    
			 
			  TPARobotDrive.frontLeftMotor.setF(0.5); //.99
			  TPARobotDrive.frontLeftMotor.setP(0.04); //.04
			  TPARobotDrive.frontLeftMotor.setI(0.0); //0
			  TPARobotDrive.frontLeftMotor.setD(0.0); //0
			     
			  TPARobotDrive.frontRightMotor.setF(0.5); //.99
			  TPARobotDrive.frontRightMotor.setP(0.04); //.04
			  TPARobotDrive.frontRightMotor.setI(0.0); //0
			  TPARobotDrive.frontRightMotor.setD(0.0); //0
		    
			     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
			     TPARobotDrive.frontLeftMotor.set(300); //1032 max speed if F is .99 and P .04
			     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
			    
			     TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
			     TPARobotDrive.frontRightMotor.set(300); //1032 max speed if F is .99 and P .04
			     TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());	  
			   


			   SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
			   SmartDashboard.putString("taskCounterB: ", "" + taskCounterB); 
		       SmartDashboard.putString("distance in the  first loop: ", "" + calculateDistance(15.0));
			   SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
			   SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
	           SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
	           SmartDashboard.putString("talon.getPosition Joe", "" + TPARobotDrive.frontLeftMotor.getPosition());
	           SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
	           SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
	           SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
	           SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
	     	   SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
		       SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());
		     
	           
		   }
		  
		   if (Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >= calculateDistance(15.0) && (taskCounterA == 0) && (taskCounterB == 0))  {
				   
			   TPARobotDrive.frontLeftMotor.setPosition(0);
			   TPARobotDrive.frontRightMotor.setPosition(0);
			    taskCounterA = 1;
			    taskCounterB = 1;
			    Timer.delay(0.1);
			    SmartDashboard.putString("distance in the second loop: ", "" + calculateDistance(15.0));
			    SmartDashboard.putString("get zero enc pos FL: ", "" +TPARobotDrive.frontLeftMotor.getPosition());
			    SmartDashboard.putString("get zero enc pos FR: ", "" +TPARobotDrive.frontRightMotor.getPosition());
			 
		  }
		  
		 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(15.0)) && (taskCounterA == 1) && (taskCounterB == 1)){
			 //talonBR.reverseOutput(true);
			 //talonFL.reverseOutput(true);
			 //talonFR.reverseOutput(true);
			 TPARobotDrive.frontLeftMotor.setF(0.5); //.99
			 TPARobotDrive.frontLeftMotor.setP(0.04); //.04
			 TPARobotDrive.frontLeftMotor.setI(0.0); //0
			 TPARobotDrive.frontLeftMotor.setD(0.0); //0
			     
			 TPARobotDrive.frontRightMotor.setF(0.5); //.99
			 TPARobotDrive.frontRightMotor.setP(0.04); //.04
			 TPARobotDrive.frontRightMotor.setI(0.0); //0
			 TPARobotDrive.frontRightMotor.setD(0.0); //0
			     
			     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
			     TPARobotDrive.frontLeftMotor.set(178); //1032 max speed if F is .99 andP .04
			     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
		         
		         TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
		         TPARobotDrive.frontRightMotor.set(178); //1032 max speed if F is .99 andP .04
		         TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());
		       
		    
		    SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
		    SmartDashboard.putString("taskCounterB: ", "" + taskCounterB);
		    SmartDashboard.putString("distance: ", "" + calculateDistance(15));
			SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
			SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
	        SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
	        SmartDashboard.putString("talon.getPosition", "" + TPARobotDrive.frontLeftMotor.getPosition());
	        SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
	        SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
	        SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
	        SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
	  	    SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
		    SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

		 }
		 
		 
		   if (Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >= calculateDistance(15.0) && (taskCounterA == 1) && (taskCounterB == 1))  {
			   
			   TPARobotDrive.frontLeftMotor.setPosition(0);
			   TPARobotDrive.frontRightMotor.setPosition(0);
			    taskCounterA = 2;
			    taskCounterB = 2;
			    Timer.delay(0.1);
			    SmartDashboard.putString("distance in the second loop: ", "" + calculateDistance(15.0));
			    SmartDashboard.putString("get zero enc pos FL: ", "" +TPARobotDrive.frontLeftMotor.getPosition());
			    SmartDashboard.putString("get zero enc pos FR: ", "" +TPARobotDrive.frontRightMotor.getPosition());
			 
		  }
		 
		 
		 
		 
		 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(15.0)) && (taskCounterA == 2) && (taskCounterB == 2)){
			 //talonBR.reverseOutput(true);
			 //talonFL.reverseOutput(true);
			 //talonFR.reverseOutput(true);
			 TPARobotDrive.frontLeftMotor.setF(0.5); //.99
			 TPARobotDrive.frontLeftMotor.setP(0.04); //.04
			 TPARobotDrive.frontLeftMotor.setI(0.0); //0
			 TPARobotDrive.frontLeftMotor.setD(0.0); //0
			     
			 TPARobotDrive.frontRightMotor.setF(0.5); //.99
			 TPARobotDrive.frontRightMotor.setP(0.04); //.04
			 TPARobotDrive.frontRightMotor.setI(0.0); //0
			 TPARobotDrive.frontRightMotor.setD(0.0); //0
			     
			     TPARobotDrive.frontLeftMotor.changeControlMode(TalonControlMode.Speed);
			     TPARobotDrive.frontLeftMotor.set(400); //1032 max speed if F is .99 andP .04
			     TPARobotDrive.backLeftMotor.set(TPARobotDrive.frontLeftMotor.getDeviceID());
		         
		         TPARobotDrive.frontRightMotor.changeControlMode(TalonControlMode.Speed);
		         TPARobotDrive.frontRightMotor.set(400); //1032 max speed if F is .99 andP .04
		         TPARobotDrive.backRightMotor.set(TPARobotDrive.frontRightMotor.getDeviceID());
		       
		    
		    SmartDashboard.putString("taskCounterA: ", "" + taskCounterA);
		    SmartDashboard.putString("taskCounterB: ", "" + taskCounterB);
		    SmartDashboard.putString("distance: ", "" + calculateDistance(15));
			SmartDashboard.putString("getEncPosition: ", "" +TPARobotDrive.frontLeftMotor.getEncPosition());
			SmartDashboard.putString("getSpeed", "" + TPARobotDrive.frontLeftMotor.getSpeed());
	        SmartDashboard.putString("talon.getEncVelocity", "" + TPARobotDrive.frontLeftMotor.getEncVelocity());
	        SmartDashboard.putString("talon.getPosition", "" + TPARobotDrive.frontLeftMotor.getPosition());
	        SmartDashboard.putString("Is safety enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isSafetyEnabled());
	        SmartDashboard.putString("Get talon Tempuratrure (Cel)", "" + TPARobotDrive.frontLeftMotor.getTemperature());
	        SmartDashboard.putString("Get Output Votage", "" + TPARobotDrive.frontLeftMotor.getOutputVoltage());
	        SmartDashboard.putString("Get current input BUS voltage (battery)", "" + TPARobotDrive.frontLeftMotor.getBusVoltage());
	  	    SmartDashboard.putString("Get Control Mode", "" + TPARobotDrive.frontLeftMotor.getControlMode());
		    SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

		 }
		 
		 
	 } // end drawBridge
  
    public void rockWall() {
    	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

    }
    
    
    // Default autonomous 
    public void lowBar() {
     SmartDashboard.putString("Entering lowBar method", " " );
     // Drive to lowbar
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(54)) && 
    		       (taskCounterA == 0) && taskCounterB == 0){
    	 
    	 makeStraight(.5);
    	 SmartDashboard.putString("Inside first if statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());
	       	 
     }
         // calculateDistance given in inches 
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >=  calculateDistance(54)) && 
		       (taskCounterA == 0) && taskCounterB == 0){
    	 SmartDashboard.putString("Inside Second if statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());
	     
    	 stopAndReset1(); 	 
     }
     // drive over lowbar
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(56)) && 
		       (taskCounterA == 1) && taskCounterB == 0){
	 
	 makeStraight(0.5);
	 SmartDashboard.putString("Inside third if statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());
     	 
     }
     
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >=  calculateDistance(56)) && 
		       (taskCounterA == 1) && taskCounterB == 0){
	 
    	 stopAndReset2(); 	 
	 SmartDashboard.putString("Inside fourth if statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());
   	 
     }
     
    			 
    } // End lowBar()
    
//    public void roughTerrain() {
 //   	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

//    }
    
//    public void sallyPort() {
//    	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());
//
//    }
    
    public void ramParts() {
    	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

    }
    
    public void portCullis() {
    	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

    }
    
    public void cheval() {
    	SmartDashboard.putString("Is Control Enabled (True or False)", "" + TPARobotDrive.frontLeftMotor.isControlEnabled());

    }
    
  // Calculates how many encoder ticks it takes to go a desired distance in inches 
  public double calculateDistance(double inches){
  	 // bot travels 23.5 inches per 1 wheel revolution, 1440 encoder ticks per 1 wheel revolution
  	 // 1440 encoder ticks divided by wheel circumf in inches 23.5
	 double encoderTicksPerInch = 1440/23.5;
  	 double targetEncValue =  (inches * encoderTicksPerInch);
  	 
  	 return targetEncValue;
  }

  public void makeStraight(double speed){
		
		if(ahrs.getAngle() > .5 || ahrs.getAngle() < 359.5){
  		
  		pid.setSetpoint(0.0f);
  		double turnDegrees;
  		pid.enable();
  		turnDegrees = pidOutput;
  		Robot.robotDrive.arcadeDrive(speed, turnDegrees * -0.5);
  		SmartDashboard.putString("ahrs.getyaw(): ", "" +ahrs.getYaw());
  		SmartDashboard.putString("ahrs.getAngle(): ", "" +ahrs.getAngle());
  		
  	}else{

  		Robot.robotDrive.arcadeDrive(speed, 0.0);
   		SmartDashboard.putString("ahrs.getyaw(): ", "" +ahrs.getYaw());
  		SmartDashboard.putString("ahrs.getAngle(): ", "" +ahrs.getAngle());
  		
  	}
 } // End makeStraight()
  
  public void stopAndReset1(){
		
		if ((taskCounterA == 0) && (taskCounterB == 0)) {
			
			Robot.robotDrive.arcadeDrive(0.0, 0.0);
			TPARobotDrive.frontLeftMotor.setPosition(0);
			SmartDashboard.putString("reseting stopAndReset1: ", "" +taskCounterA);
			taskCounterA = 1;
			Timer.delay(.1);
			
		}
		
	    if((taskCounterA == 1) && (taskCounterB == 0)) {
			
			Robot.robotDrive.arcadeDrive(0.0, 0.0);
			TPARobotDrive.frontLeftMotor.setPosition(0);
			SmartDashboard.putString("reseting stopAndReset1: ", "" +taskCounterA);
			taskCounterA = 2;
			Timer.delay(.1);
			
		}
		
	  
	} // End stopAndReset1()
  
    public void stopAndReset2() {
    	  if(taskCounterA == 1){
  			
  			Robot.robotDrive.arcadeDrive(0.0, 0.0);
  			TPARobotDrive.frontLeftMotor.setPosition(0);
  			SmartDashboard.putString("reseting stopAndReset2: ", "" +taskCounterA);
  			taskCounterA = 2;
  			Timer.delay(.1);
  			
  		}
    	
    }
  
  
  
@Override
public void pidWrite(double output) {
	pidOutput = output;
	
}
    
    
    
} // End Autotasks 

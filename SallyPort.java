package org.usfirst.frc.team3944.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SallyPort implements PIDOutput{
	
	private int taskCounterA = 0;
    private int taskCounterB = 0;
       
    // Orchestration Variables (all distance in inches) 
    private double _distanceTask1 = 69; 	// From Start to low bar in inches
    private double _distanceTask2 = 100;  // Drive over low bar platform
    private double _speed1Task1	 = 0.7;
    private double _speed1Task2	 = 0.7;
    private double _minAngleRotationTask3 = 44.75;
    private double _maxAngleRotationTask3 = 45.5;
    private double _distanceTask4 = 151;   // From turn point to shooting point in inches
    private double _speed1Task4  = 0.7;
     
    // turn method variables 
    private float  _rotationAngleTurn1 	= 45.0f;
    private double _governorTurn1 		= -0.6;
    
    // makeStraight method variables 
    private double _leftRotationAngleBoundaryMS  	= 359.50; 
    private double _rightRotationAngleBoundaryMS 	= 1.50; 
    private double _governorZeroingSpeedMS			= -0.7;

    // alignmentVector variables 
    private double _leftRotationAngleBoundaryAV 	= 44.75;
    private double _rightRotationAgnleBoundaryAV 	= 45.5;
    private double _governorZeroingSpeedAV			= -0.7;
    private float  _pidSetPointAV					= 45.0f;
    
	private PIDController pid;
	private static double pidOutput;
	private AHRS ahrs;
	
	public SallyPort(AHRS ahrs) {
    	this.ahrs = ahrs; 
        pid = new PIDController(0.03, 0.1, 0.1, 0.01, ahrs, this);
        pid.setInputRange(-180.0f,  180.0f);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsoluteTolerance(1.0f);
        pid.setContinuous(true);	
        ahrs.reset();
        taskCounterA = 0;
        TPARobotDrive.frontLeftMotor.setPosition(0); 
    }
    
	// Default autonomous 
    public void startTask() {
     SmartDashboard.putString("Entering Low Bar method", " " );
     // Task1 drive to lowbar from starting position in a straight line
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(_distanceTask1)) && 
    		       (taskCounterA == 0)){ 
    	 makeStraight(_speed1Task1);
    	 SmartDashboard.putString("Task1 first IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());       	 
     }
     // After bot reaches desired distance stop and reset encoder and increment counter
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >=  calculateDistance(_distanceTask1)) && 
		       (taskCounterA == 0)){
    	 SmartDashboard.putString("Task1 Second IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());	     
    	 stopAndReset1(); 	 
     }
     
     // Task2 drive over low bar platform
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(_distanceTask2)) && 
		       (taskCounterA == 1)){	 
    	 makeStraight(_speed1Task2);
    	 SmartDashboard.putString("Task2 First IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());   	 
     }
     // After bot reaches desired distance stop and reset encoder and increment counter
     if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >=  calculateDistance(_distanceTask2)) && 
		       (taskCounterA == 1)){	 
    	 stopAndReset2(); 	 
    	 SmartDashboard.putString("Task2 Second IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());  	 
     }
         
     // Task3 Turn 45 degrees 
 	 if(taskCounterA == 2){	
		turn1(taskCounterA);		
	 }
 	 // Uses NavX to set 45 degree angle, then stops and rests encoders 
 	 if(ahrs.getAngle() > _minAngleRotationTask3 && ahrs.getAngle() < _maxAngleRotationTask3 && 
 			 	(taskCounterA == 2)){		
		stopAndReset3();		
	 }	
 	 
 	 // Task4 drive to shooting position 
 	 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) <  calculateDistance(_distanceTask4)) && 
		       (taskCounterA == 3)){	 
 		 alignmentVector(_speed1Task4);
 		 SmartDashboard.putString("Task4 First IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());   	 
 	 }
 	 // After bot reaches desired distance stop and reset encoder and increment counter
 	 if ((Math.abs(TPARobotDrive.frontLeftMotor.getPosition()) >=  calculateDistance(_distanceTask4)) && 
		       (taskCounterA == 3)){	 
 		 stopAndReset4(); 	 
 		 SmartDashboard.putString("Task4 Second IF statement encoder value ", "" +TPARobotDrive.frontLeftMotor.getPosition());  	 
 	 }
 	 	 
    } // End startTask
    
       
    // Calculates how many encoder ticks it takes to go a desired distance in inches 
    public double calculateDistance(double inches){
    	 // bot travels 23.5 inches per 1 wheel revolution, 1440 encoder ticks per 1 wheel revolution
    	 // 1440 encoder ticks divided by wheel circumf in inches 23.5
  	     double encoderTicksPerInch = 1440/23.5;
    	 double targetEncValue =  (inches * encoderTicksPerInch);
    	 
    	 return targetEncValue;
    }  // End calculateDistance
    
    // Enables robot to drive in a straight line
    public void makeStraight(double speed){
  		
  		if(ahrs.getAngle() < _leftRotationAngleBoundaryMS || ahrs.getAngle() > _rightRotationAngleBoundaryMS){ 		
    		pid.setSetpoint(0.0f);
    		double turnDegrees;
    		pid.enable();
    		turnDegrees = pidOutput;
    		Robot.robotDrive.arcadeDrive(speed, turnDegrees * _governorZeroingSpeedMS);
    		SmartDashboard.putString("ahrs.getyaw(): ", "" +ahrs.getYaw());
    		SmartDashboard.putString("ahrs.getAngle(): ", "" +ahrs.getAngle());  		
    	}else{
    		Robot.robotDrive.arcadeDrive(speed, 0.0);
     		SmartDashboard.putString("ahrs.getyaw(): ", "" +ahrs.getYaw());
    		SmartDashboard.putString("ahrs.getAngle(): ", "" +ahrs.getAngle());   		
    	}
   } // End makeStraight()
    
    // Executes turn in Task3 
	public void turn1(int counter){		
		if(counter == taskCounterA){  		
    		pid.setSetpoint(_rotationAngleTurn1);
    		double turn1Degrees;
    		pid.enable();
    		turn1Degrees = pidOutput;
    		Robot.robotDrive.arcadeDrive(0.0, turn1Degrees * _governorTurn1); 		
    	}
	}  // End turn
	
	
	public void alignmentVector(double speed) {
	 if(ahrs.getAngle() < _leftRotationAngleBoundaryAV || ahrs.getAngle() > _rightRotationAgnleBoundaryAV && taskCounterA == 3){		
		pid.setSetpoint(_pidSetPointAV);
		double alignmentDegrees;
		pid.enable();
		alignmentDegrees = pidOutput;
		Robot.robotDrive.arcadeDrive(speed, alignmentDegrees * _governorZeroingSpeedAV);		
	 }else{
		Robot.robotDrive.arcadeDrive(speed, 0.0);
	 }
	
	} // End alignmentVector
	
	
// Stop and reset methods     
    public void stopAndReset1(){	
  		if (taskCounterA == 0) { 			
  			Robot.robotDrive.arcadeDrive(0.0, 0.0);
  			TPARobotDrive.frontLeftMotor.setPosition(0);
  			SmartDashboard.putString("reseting stopAndReset1: ", "" +taskCounterA);
  			taskCounterA = 1;
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
     } // End stopAndReset2
      
      
     public void stopAndReset3() {
    	 if(taskCounterA == 2){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontLeftMotor.setPosition(0);
    		SmartDashboard.putString("reseting stopAndReset3: ", "" +taskCounterA);
    		taskCounterA = 3;
    		Timer.delay(.1);   			
    	 }    	
     } // End stopAndReset3
     
     public void stopAndReset4() {
    	 if(taskCounterA == 3){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontLeftMotor.setPosition(0);
    		SmartDashboard.putString("reseting stopAndReset4: ", "" +taskCounterA);
    		taskCounterA = 4;
    		Timer.delay(.1);   			
    	 }    	
     } // End stopAndReset4
    

	@Override
	public void pidWrite(double output) {
		pidOutput = output;
		
	}

} // End SallyPort

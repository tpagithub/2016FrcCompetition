package org.usfirst.frc.team3944.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LowBar implements PIDOutput {
	
    private int taskCounterA = 0;
    private int taskCounterB = 0;
    
    private final double distanceTask1Const = 73;       	// DIST-TASK-1
    private final double distanceTask2Const = 47;       	// DIST-TASK-2
    private final double task3Distance = 88;		//lowbar going to the pivot point
    private final double task5Distance = 60;			//lowbar going driving to the lip of the batter
    private final double task7Distance = 50;		//lowbar going to the middle
    private final double task9Distance = 12;		//lowbar going to the lip of the middle batter
    private final double task11Distance = 50;       //lowbar going to the right goal
    private final double task13Distance = 24; 		//lowbar going to the right goal
    private final double task15Distance = 1;		//lowbar going to the lip of the right batter
//    private final double lowBarDistance = 15;		//drives to the lip of the lowbar
       
    // Orchestration Variables (all distance in inches)
    /*
    private double _distanceTask1 = 54; 	// From Start to low bar in inches
    private double _distanceTask2 = 66;   // Drive over low bar platform
    private double _speed1Task1	 = 1.0;
    private double _speed1Task2	 = .57;
    private double _minAngleRotationTask4 = 29.75;
    private double _maxAngleRotationTask4 = 30.50;
    private double _distanceTask4 = 151;   // From turn point to shooting point in inches
    private double _speed1Task4  = 0.5;
    */
    
    // speed variables 
    private final double speedTask1Const = 1.0;
    private final double speedTask2Const = 0.57;
    private final double task3Speed = 0.8; 
    private final double task5Speed = 0.65;
    private final double task7Speed = 1.0;
    private final double task9Speed = 1.0;
    private final double task11Speed = 1.0;
    private final double task13Speed = 1.0;
    private final double task15Speed = 1.0;
    private final double pos6Task1Speed = 1.0;
    private final double pos7Task1Speed = 0.7;
    
    // turn method variables 
    private float  _rotationAngleTurn1 	= 30.0f;
    private double _governorTurn1 		= -0.75;
    
    // makeStraight method variables 
    private double _leftRotationAngleBoundaryMS  	= 359.50; 
    private double _rightRotationAngleBoundaryMS 	= 1.50;
    private double _governorZeroingSpeedMS			= -0.57;

    // alignmentVector variables 
    private double _leftRotationAngleBoundaryAV 	= 44.75;
    private double _rightRotationAgnleBoundaryAV 	= 45.5;
    private double _governorZeroingSpeedAV			= -0.7;
    private float  _pidSetPointAV					= 45.0f;
    
	private PIDController pid;
	private static double pidOutput;
	private AHRS ahrs;
	
    public LowBar(AHRS ahrs) {
    	this.ahrs = ahrs; 
        pid = new PIDController(0.03, 0.1, 0.1, 0.01, ahrs, this);
        pid.setInputRange(-180.0f,  180.0f);
        pid.setOutputRange(-1.0, 1.0);
        pid.setAbsoluteTolerance(1.0f);
        pid.setContinuous(true);	
        ahrs.reset();
        taskCounterA = 0;
        TPARobotDrive.frontRightMotor.setPosition(0); 
    }
    
    
    // Default autonomous 
    public void startTask() {
     //first position:
     if(Robot.switchAPosition[0] == true && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
  		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){
     
     // Task1 drive to lowbar from starting position in a straight line
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(distanceTask1Const)) && 
    		       (taskCounterA == 0)){ 
    	 makeStraight(speedTask1Const);   	 
     }
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(distanceTask1Const)) && 
		       (taskCounterA == 0)){
    	 stopAndReset1(); 	 
     }
     
     // Task2 drive over low bar platform
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(distanceTask2Const)) && 
		       (taskCounterA == 1)){	 
    	 makeStraight(speedTask2Const);   	 
     }
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(distanceTask2Const)) && 
		       (taskCounterA == 1)){	 
    	 stopAndReset2(); 	  
     }

     if(Robot.towerState == Robot.TOWER_POS_1 && Robot.towerState != Robot.TOWER_POS_2 
    		 && Robot.towerState != Robot.TOWER_POS_3 && taskCounterA == 2){
    	//CODE FOR LEFT GOAL
		startShooter();
		checkLoader();
		stopAndReset3();
     }
     if(Robot.towerState != Robot.TOWER_POS_1 && Robot.towerState == Robot.TOWER_POS_2 
    		 && Robot.towerState != Robot.TOWER_POS_3 && taskCounterA == 2){
    	//CODE FOR MIDDLE GOAL 
    	startShooter();
 		checkLoader();
    	stopAndReset7();
     }
     if(Robot.towerState != Robot.TOWER_POS_1 && Robot.towerState != Robot.TOWER_POS_2 
    		 && Robot.towerState == Robot.TOWER_POS_3 && taskCounterA == 2){
    	//CODE FOR RIGHT GOAL
     	startShooter();
 		checkLoader();
 		stopAndReset12();
     }
     
     // Task3 drive until parallel with the tower                  
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task3Distance)) && 
		       (taskCounterA == 3)){ 
     	 startShooter();
  		 checkLoader();
    	 makeStraight(task3Speed);        
     }
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task3Distance)) && 
	       (taskCounterA == 3)){
     	 startShooter();
  		 checkLoader();
    	 stopAndReset4();   	 
     }
     
     // Task4 Turn 60 degrees 
 	 if(taskCounterA == 4){	
     	startShooter();
 		checkLoader();
 		turn1(taskCounterA, 60.0f);	
	 }
 	 if(ahrs.getAngle() > 59.95 && ahrs.getAngle() < 60.25 && 
 			 	(taskCounterA == 4)){	
        startShooter();
     	checkLoader();
 		stopAndReset5();	
	 }	
 	 
 	 // Task5 drive to the left shooting position                    
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task5Distance)) && 
		       (taskCounterA == 5)){	 
     	startShooter();
 		checkLoader();
 		alignmentVector(task5Speed,60.0f,taskCounterA);   // task5Speed 0.75
 		   	 
 	 }
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task5Distance)) && 
		       (taskCounterA == 5)){
     	 startShooter();
  		 checkLoader();
 		 stopAndReset6(); 	   	 
 	 }

 	 /*
 	  * MIDDLE TOWER
 	  */
     // Task6 rotate 90 degrees making the robot perpendicular to the middle tower
 	 if(taskCounterA == 6){	 	   
     	startShooter();
		turn1(taskCounterA, 90.0f);
		checkLoader();

	 }
 	 if(ahrs.getAngle() > 89.75 && ahrs.getAngle() < 90.5 && 
 			 	(taskCounterA == 6)){
     	startShooter();
 		checkLoader();
 		stopAndReset8();	
	 }
     
 	 // Task7 drive to the middle tower                              
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task7Distance)) && 
		       (taskCounterA == 7)){	 
     	startShooter();
 		checkLoader();  	 
 		alignmentVector(task7Speed,90.0f,taskCounterA);  
 	 }
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task7Distance)) && 
		       (taskCounterA == 7)){
     	startShooter();
  		checkLoader();
 		stopAndReset9(); 	   	 
 	 }
     
     // Task8 rotate to 0 degrees to be parallel with the middle tower
 	 if(taskCounterA == 8){	
     	startShooter();
  		checkLoader();
  		turn1(taskCounterA, 0.0f);	
	 } 
 	 if(ahrs.getAngle() > 359.75 && (taskCounterA == 8)|| ahrs.getAngle() < 0.5 && 
 			 	(taskCounterA == 8)){
 		startShooter();
 		checkLoader();
 		stopAndReset10();	
	 }

     // Task9 drive to the middle tower to shoot                   
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task9Distance)) && 
    		       (taskCounterA == 9)){ 
     	 startShooter();
    	 checkLoader();       	 
    	 makeStraight(task9Speed);     
     }
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task9Distance)) && 
		       (taskCounterA == 9)){	
     	startShooter();
  		checkLoader();
    	stopAndReset11(); 	 
     }
     
     /*
      * RIGHT TOWER
      */
     // Task10 rotate 90 degrees making the robot perpendicular to the left tower
 	 if(taskCounterA == 10){	 	   
     	startShooter();
     	checkLoader();
		turn1(taskCounterA, 90.0f);

	 } 
 	 if(ahrs.getAngle() > 89.75 && ahrs.getAngle() < 90.5 && 
 			 	(taskCounterA == 10)){
      	startShooter();
      	checkLoader();
 		stopAndReset13();	
	 }
     
 	 // Task11 drive to the left tower                                
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task11Distance)) && 
		       (taskCounterA == 11)){	 
      	startShooter();
      	checkLoader();
 		alignmentVector(task11Speed,90.0f,taskCounterA);   	  
 	 }
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task11Distance)) && 
		       (taskCounterA == 11)){
      	startShooter();
      	checkLoader();
 		stopAndReset14(); 	   	 
 	 }
     
 	 //Task12 rotate so parallel the right tower
 	 if(taskCounterA == 12){	
      	startShooter();
      	checkLoader();
 	 	turn1(taskCounterA, 0.0f);		
	 }
	 if(ahrs.getAngle() > 359.75 && (taskCounterA == 12)|| ahrs.getAngle() < 0.5 && 
			 	(taskCounterA == 12)){
	    startShooter();
	    checkLoader();
		stopAndReset15();	
	 }
	 
     // Task13 drive until parallel with the tower					 
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task13Distance)) && 
		       (taskCounterA == 13)){ 
      	startShooter();
      	checkLoader();
    	makeStraight(task13Speed);   	  
     }
     if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task13Distance)) && 
	       (taskCounterA == 13)){
      	startShooter();
      	checkLoader();    
    	stopAndReset16(); 	 
     }
     
     // Task14 Turn -60(yaw i.e to the left) degrees 
 	 if(taskCounterA == 14){	
      	startShooter();
      	checkLoader();
 		turn1(taskCounterA, -60.0f);
	 }
 	 if(ahrs.getAngle() > 299.75 && ahrs.getAngle() < 300.50 && 
 			 	(taskCounterA == 14)){	
      	startShooter();
      	checkLoader();
 		stopAndReset17();	
	 }	
 	 
 	 // Task15 drive to the right tower shooting position         task15Distance  14
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(task15Distance)) && 
		       (taskCounterA == 15)){	 
      	startShooter();
      	checkLoader();
 		alignmentVector(task15Speed,-60.0f,taskCounterA);    
 		   	 
 	 }
 	 if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(task15Distance)) && 
		       (taskCounterA == 15)){
      	startShooter();
      	checkLoader();
 		stopAndReset18(); 	   	 
 	 }

 	 /*
 	  * SHOOT!
 	  */
     if(taskCounterA == 16 && TPAShooter.masterShooter.getSpeed() >= TPARobotMap.shooterOptimalSpeed2){
		 TPALoader.loaderMotor.setInverted(false);
		 TPALoader.loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
		 TPALoader.loaderMotor.set(1.0); 
     }
     if(Robot.loaderSwitch.get() == true && taskCounterA == 16){
  		TPAShooter.masterShooter.set(0.0);
  		TPAShooter.slaveShooter.set(TPAShooter.masterShooter.getDeviceID());
  		TPALoader.loaderMotor.setInverted(false);
		TPALoader.loaderMotor.changeControlMode(TalonControlMode.PercentVbus);
		TPALoader.loaderMotor.set(0.0);
     }
     }
     //second position:
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == true && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){

     }
     //third position:
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == true &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){

     }
     //fourth position:
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == true && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){

     }
     //fifth position:
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == true && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){
    	 
     } 
     
     //sixth position: drives over the lowbar for 10 points
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == true &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == false){
        /* if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(distanceTask1Const)) && 
  		       (taskCounterA == 0)){ 
        	 makeStraight(pos6Task1Speed);   	 
         }
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(distanceTask1Const)) && 
		       (taskCounterA == 0)){
        	 stopAndReset1(); 	 
         }
   
         // Task2 drive over low bar platform
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(11000-distanceTask1Const)) && 
		       (taskCounterA == 1)){	 
        	 makeStraight(speedTask2Const);   	 
         }
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(1100-distanceTask1Const)) && 
		       (taskCounterA == 1)){	 
        	 stopAndReset2(); 	  
         }*/
    	//lowbar distance 												 
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  13000) && 
    		       (taskCounterA == 0)){ 
          	 makeStraight(pos6Task1Speed);   	 
         }
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  13000) && 
  		       (taskCounterA == 0)){
        	 stopAndReset1(); 	 
         }
     }
     
     //seventh position: drives to the low bar for 2 points 
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == true && Robot.switchAPosition[7] == false){
    	 //lowbar distance 												 
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) <  calculateDistance(distanceTask1Const)) && 
    		       (taskCounterA == 0)){ 
          	 makeStraight(pos7Task1Speed);   	 
         }
         if ((Math.abs(TPARobotDrive.frontRightMotor.getPosition()) >=  calculateDistance(distanceTask1Const)) && 
  		       (taskCounterA == 0)){
        	 stopAndReset1(); 	 
         }
     }
     
     //eighth position: do nothing 
     if(Robot.switchAPosition[0] == false && Robot.switchAPosition[1] == false && Robot.switchAPosition[2] == false &&
    		 Robot.switchAPosition[3] == false && Robot.switchAPosition[4] == false && Robot.switchAPosition[5] == false &&
    		 Robot.switchAPosition[6] == false && Robot.switchAPosition[7] == true){
    	 
     }
     
    } // End startLB
    
       
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
	public void turn1(int counter, float angleOfRotation){		
		if(counter == taskCounterA){  		
    		pid.setSetpoint(angleOfRotation);
    		double turn1Degrees;
    		pid.enable();
    		turn1Degrees = pidOutput;
    		Robot.robotDrive.arcadeDrive(0.0, turn1Degrees * _governorTurn1); 		
    	}
	}  // End turn
	
	//DO NOT USE FOR 0
	public void alignmentVector(double speed, float angleOfAlignment, int counter) {
	 if(angleOfAlignment < 0){
		double absoluteAngle = 360 - (Math.abs((double) angleOfAlignment)); 
	 if(ahrs.getAngle() < ((double) absoluteAngle - .5) || ahrs.getAngle() > ((double) absoluteAngle + .15)
			 && counter == taskCounterA){		
		pid.setSetpoint(angleOfAlignment);
		double alignmentDegrees;
		pid.enable();
		alignmentDegrees = pidOutput;
		Robot.robotDrive.arcadeDrive(speed, alignmentDegrees * _governorZeroingSpeedAV);		
	 }else{
		Robot.robotDrive.arcadeDrive(speed, 0.0);
	 }
	 }
	 
	 if(angleOfAlignment > 0){
	 if(ahrs.getAngle() < ((double) angleOfAlignment - .5) || ahrs.getAngle() > ((double) angleOfAlignment + .15)
			 && counter == taskCounterA){		
		pid.setSetpoint(angleOfAlignment);
		double alignmentDegrees;
		pid.enable();
		alignmentDegrees = pidOutput;
		Robot.robotDrive.arcadeDrive(speed, alignmentDegrees * _governorZeroingSpeedAV);		
	 }else{
		Robot.robotDrive.arcadeDrive(speed, 0.0);
	 }
	 }
	 
	} // End alignmentVector
	
	
// Stop and reset methods     
    public void stopAndReset1(){	
  		if (taskCounterA == 0) { 			
  			Robot.robotDrive.arcadeDrive(0.0, 0.0);
  			TPARobotDrive.frontRightMotor.setPosition(0);
  			SmartDashboard.putString("reseting stopAndReset1: ", "" +taskCounterA);
  			taskCounterA = 1;
  			Timer.delay(.1);			
  		}  
  	} // End stopAndReset1()
    
    public void stopAndReset2() {
    	if(taskCounterA == 1){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
    		SmartDashboard.putString("reseting stopAndReset2: ", "" +taskCounterA);
   			taskCounterA = 2;
   			Timer.delay(.1);  			
    	}     	
     } // End stopAndReset2
      
    
     public void stopAndReset3() {
    	 if(taskCounterA == 2){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
    		SmartDashboard.putString("reseting stopAndReset3: ", "" +taskCounterA);
    		taskCounterA = 3;
    		Timer.delay(.1);   			
    	 }    	
     } // End stopAndReset3
     
    
     public void stopAndReset4() {
    	 if(taskCounterA == 3){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
    		SmartDashboard.putString("reseting stopAndReset4: ", "" +taskCounterA);
    		taskCounterA = 4;
    		Timer.delay(.1);   			
    	 }    	
     } // End stopAndReset4
    
    
    public void stopAndReset5() {
    	if(taskCounterA == 4){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset5: ", "" +taskCounterA);
   			taskCounterA = 5;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset5
    
    public void stopAndReset6() {
    	if(taskCounterA == 5){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset6: ", "" +taskCounterA);
   			taskCounterA = 16;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset6
    
    public void stopAndReset7() {
    	if(taskCounterA == 2){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset7: ", "" +taskCounterA);
   			taskCounterA = 6;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset7
    
    public void stopAndReset8() {
    	if(taskCounterA == 6){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset8: ", "" +taskCounterA);
   			taskCounterA = 7;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset8
    
    public void stopAndReset9() {
    	if(taskCounterA == 7){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset9: ", "" +taskCounterA);
   			taskCounterA = 8;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset9
    
    public void stopAndReset10() {
    	if(taskCounterA == 8){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset10: ", "" +taskCounterA);
   			taskCounterA = 9;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset10
    
    public void stopAndReset11() {
    	if(taskCounterA == 9){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset11: ", "" +taskCounterA);
   			taskCounterA = 16;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset11
    
    public void stopAndReset12() {
    	if(taskCounterA == 2){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset12: ", "" +taskCounterA);
   			taskCounterA = 10;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset12
    
    public void stopAndReset13() {
    	if(taskCounterA == 10){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset13: ", "" +taskCounterA);
   			taskCounterA = 11;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset13
    
    public void stopAndReset14() {
    	if(taskCounterA == 11){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset14: ", "" +taskCounterA);
   			taskCounterA = 12;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset14
    
    public void stopAndReset15() {
    	if(taskCounterA == 12){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset15: ", "" +taskCounterA);
   			taskCounterA = 13;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset15
    
    public void stopAndReset16() {
    	if(taskCounterA == 13){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset16: ", "" +taskCounterA);
   			taskCounterA = 14;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset16
    
    public void stopAndReset17() {
    	if(taskCounterA == 14){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset17: ", "" +taskCounterA);
   			taskCounterA = 15;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset17
    
    public void stopAndReset18() {
    	if(taskCounterA == 15){    			
    		Robot.robotDrive.arcadeDrive(0.0, 0.0);
    		TPARobotDrive.frontRightMotor.setPosition(0);
   			SmartDashboard.putString("reseting stopAndReset18: ", "" +taskCounterA);
   			taskCounterA = 16;
   			Timer.delay(.1);   			
    	}    	
    } // End stopAndReset18
    

    
    public void startShooter(){
 		TPAShooter.masterShooter.set(TPARobotMap.shooterOptimalSpeed2);
     	TPAShooter.slaveShooter.set(TPAShooter.masterShooter.getDeviceID()); 
     	SmartDashboard.putString("AUTOSHOOTERSPEED: ", ""+TPAShooter.masterShooter.getSpeed());
    }
    public void checkLoader(){
    	if(Robot.loaderSwitch.get() == true){
    		TPALoader.loaderMotor.set(1.0);
    	}
    	if(Robot.loaderSwitch.get() == false){
    		TPALoader.loaderMotor.set(0.0);
    	}
    }
	@Override
	public void pidWrite(double output) {
		pidOutput = output;
		
	}

} // End lowBar

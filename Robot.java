
package org.usfirst.frc.team3944.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.networktables2.type.BooleanArray;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import edu.wpi.first.wpilibj.RobotDrive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
  
    
    // Declare TPA Objects
    private TPAJoystick joystick;
   // private TPARobotDrive robotDrive;
    public static TPARobotDrive robotDrive;
    private TPAAnalogInput analogSwitchA, analogSwitchB, analogSwitchC; // position, defense, tower;

    private TPALoader loader;
    private TPAShooter shooter;
    private TPALifter lifter;
    private DigitalInput frontLiftSwitch, backLiftSwitch;
    public static DigitalInput loaderSwitch;
    private TPAShooterLight light;
    //private NavXTeleopAssist navxTeleopAssist;
    
    //Declare Autonomous Variables
    
    // correspond to 8 switch positions
    public static boolean[] switchAPosition = new boolean[8];
   
    private TPAAutoTasks autoTasks;
    private AHRS ahrs;
    private LowBar lowBar;
    private RoughTerrain roughTerrain;
    private SallyPort sallyPort;
    private RamParts ramParts;
    private RockWall rockWall;
    private Moat moat;
    
    public static int positionState;
    public static int defenseState;
    public static int towerState;
    
    // List of possible states    
    public final static int AUTON_POS_1 = 1;
    public final static int AUTON_POS_2 = 2;
    public final static int AUTON_POS_3 = 3;
    public final static int AUTON_POS_4 = 4;
    public final static int AUTON_POS_5 = 5;
    public final static int AUTON_POS_6 = 6;
    public final static int AUTON_POS_7 = 7;
    public final static int AUTON_POS_8 = 8;
    
    public final static int DEFENSE_1 = 1;
    public final static int DEFENSE_2 = 2;
    public final static int DEFENSE_3 = 3;
    public final static int DEFENSE_4 = 4;
    public final static int DEFENSE_5 = 5;
    public final static int DEFENSE_6 = 6;
    public final static int DEFENSE_7 = 7;
    public final static int DEFENSE_8 = 8;
    
    public final static int TOWER_POS_1 = 1;
    public final static int TOWER_POS_2 = 2;
    public final static int TOWER_POS_3 = 3;
    public final static int TOWER_POS_4 = 4;
    public final static int TOWER_POS_5 = 5;
    public final static int TOWER_POS_6 = 6;
    public final static int TOWER_POS_7 = 7;
    public final static int TOWER_POS_8 = 8;
    private int c;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
     // Constructor Instantiation
     joystick = new TPAJoystick(TPARobotMap.JoystickPort);	
     robotDrive = new TPARobotDrive(TPARobotDrive.frontLeftMotor,
	 		 	TPARobotDrive.backLeftMotor,
	 			TPARobotDrive.frontRightMotor,
				TPARobotDrive.backRightMotor,
				joystick);    
        
        
     loaderSwitch = new DigitalInput(TPARobotMap.loaderLimitSwitch_ID);
     frontLiftSwitch = new DigitalInput(TPARobotMap.lifterFrontLimitSwitch);
     backLiftSwitch = new DigitalInput(TPARobotMap.lifterBackLimitSwitch);
        
        
    
     loader = new TPALoader(joystick, loaderSwitch);
     shooter = new TPAShooter(joystick, loaderSwitch);
     lifter = new TPALifter(joystick, frontLiftSwitch, backLiftSwitch);
     analogSwitchA = new TPAAnalogInput(0);
     analogSwitchB = new TPAAnalogInput(1);
     analogSwitchC = new TPAAnalogInput(2);  
     light = new TPAShooterLight(0,joystick);
     TPARobotDrive.frontRightMotor.setPosition(0);
    }
    
	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the switch structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
    public void autonomousInit() {
    	 try {
    	        /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
    	        /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
    	        /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
    	        
    	      ahrs = new AHRS(SPI.Port.kMXP);
    	      autoTasks = new TPAAutoTasks(ahrs);
    	      lowBar = new LowBar(ahrs);
    	      roughTerrain = new RoughTerrain(ahrs);
    	      sallyPort = new SallyPort(ahrs);
    	      ramParts = new RamParts(ahrs);
    	      rockWall = new RockWall(ahrs);
    	      moat = new Moat(ahrs);
    	      
    	     } catch (RuntimeException ex ) {
    	        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    	     } // End try
    	
    	//TPARobotDrive.frontLeftMotor.setPosition(0);   

		  // Initialize autonomous switch position boolean array to false
        for(int i=0; i<switchAPosition.length; i++) {
        	switchAPosition[i] = false;
        }
		
      //position 1: Voltage between 0 and 0.6
        if(analogSwitchA.getVoltage() > 0.0 && analogSwitchA.getVoltage() < /*0.007*/0.500)  {
        	positionState = AUTON_POS_1;
    	  SmartDashboard.putNumber("Get voltage SwitchA Pos1", analogSwitchA.getVoltage());
    	  SmartDashboard.putString("The current positionState is: ", "" + positionState);
    	  lowBar.startTask();
        }
        else if (analogSwitchA.getVoltage() >= /*0.007*/0.500 && analogSwitchA.getVoltage() < /*0.700*/0.800)  {
        	positionState = AUTON_POS_2;
          SmartDashboard.putNumber("Get voltage SwitchA Pos2", analogSwitchA.getVoltage());
          SmartDashboard.putString("The current positionState is: ", "" + positionState);
        }
        else if (analogSwitchA.getVoltage() >= /*0.700*/0.800 && analogSwitchA.getVoltage() < /*1.400*/1.500)  {
        	positionState = AUTON_POS_3;
            SmartDashboard.putNumber("Get voltage SwitchA Pos3", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        else if (analogSwitchA.getVoltage() >= /*1.400*/1.500 && analogSwitchA.getVoltage() < /*2.100*/2.200)  {
        	positionState = AUTON_POS_4;
            SmartDashboard.putNumber("Get voltage SwitchA Pos4", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        else if (analogSwitchA.getVoltage() >= /*2.100*/2.200 && analogSwitchA.getVoltage() < /*2.800*/2.900)  {
        	positionState = AUTON_POS_5;
            SmartDashboard.putNumber("Get voltage SwitchA Pos5", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        else if (analogSwitchA.getVoltage() >= /*2.800*/2.900 && analogSwitchA.getVoltage() < /*3.500*/3.600)  {
        	positionState = AUTON_POS_6;
            SmartDashboard.putNumber("Get voltage SwitchA Pos6", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        else if (analogSwitchA.getVoltage() >= /*3.500*/3.600 && analogSwitchA.getVoltage() < /*4.200*/4.300)  {
        	positionState = AUTON_POS_7;
            SmartDashboard.putNumber("Get voltage SwitchA Pos7", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        else if (analogSwitchA.getVoltage() >= /*4.200*/4.300 && analogSwitchA.getVoltage() < /*4.900*/5.000)  {
        	positionState = AUTON_POS_8;
            SmartDashboard.putNumber("Get voltage SwitchA Pos8", analogSwitchA.getVoltage());
            SmartDashboard.putString("The current positionState is: ", "" + positionState);
          }
        
        // Defenses Switch B
        if(analogSwitchB.getVoltage() > 0.0 && analogSwitchB.getVoltage() < 0.007)  {
      	  	  defenseState = DEFENSE_1;
      	  	  SmartDashboard.putNumber("Get voltage SwitchB Pos1", analogSwitchB.getVoltage());
      	  	  SmartDashboard.putString("The current defense is: ", "" + defenseState);
          }
          else if (analogSwitchB.getVoltage() >= 0.007 && analogSwitchB.getVoltage() < /*0.700*/0.800)  {
        	  defenseState = DEFENSE_2;
        	  SmartDashboard.putNumber("Get voltage SwitchB Pos2", analogSwitchB.getVoltage());
        	  SmartDashboard.putString("The current defense is: ", "" + defenseState);
          }
          else if (analogSwitchB.getVoltage() >= /*0.700*/0.800 && analogSwitchB.getVoltage() < /*1.400*/1.500)  {
        	  defenseState = DEFENSE_3;
              SmartDashboard.putNumber("Get voltage SwitchB Pos3", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }
          else if (analogSwitchB.getVoltage() >= /*1.400*/1.500 && analogSwitchB.getVoltage() < /*2.100*/2.200)  {
        	  defenseState = DEFENSE_4;
              SmartDashboard.putNumber("Get voltage SwitchB Pos4", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }     
          else if (analogSwitchB.getVoltage() >= /*2.100*/2.200 && analogSwitchB.getVoltage() < /*2.800*/2.900)  {
        	  defenseState = DEFENSE_5;
              SmartDashboard.putNumber("Get voltage SwitchB Pos5", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }
          else if (analogSwitchB.getVoltage() >= /*2.800*/2.900 && analogSwitchB.getVoltage() < /*3.500*/3.600)  {
        	  defenseState = DEFENSE_6;
              SmartDashboard.putNumber("Get voltage SwitchB Pos6", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }
          else if (analogSwitchB.getVoltage() >= /*3.500*/3.600 && analogSwitchB.getVoltage() < /*4.200*/4.300)  {
        	  defenseState = DEFENSE_7;
              SmartDashboard.putNumber("Get voltage SwitchB Pos7", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }
          else if (analogSwitchB.getVoltage() >= /*4.200*/4.300 && analogSwitchB.getVoltage() < /*4.900*/5.000)  {
        	  defenseState = DEFENSE_8;
              SmartDashboard.putNumber("Get voltage SwitchB Pos8", analogSwitchB.getVoltage());
              SmartDashboard.putString("The current defense is: ", "" + defenseState);
            }
        
     // Tower Position Switch C
        if(analogSwitchC.getVoltage() > 0.0 && analogSwitchC.getVoltage() < 0.007)  {
        	  towerState = TOWER_POS_1;
        	  SmartDashboard.putNumber("Get voltage SwitchC Pos1", analogSwitchC.getVoltage());
        	  SmartDashboard.putString("The current towerState is: ", "" + towerState);
          }
          else if (analogSwitchC.getVoltage() >= 0.007 && analogSwitchC.getVoltage() < /*0.700*/0.800)  {
        	  towerState = TOWER_POS_2;
        	  SmartDashboard.putNumber("Get voltage SwitchC Pos2", analogSwitchC.getVoltage());
        	  SmartDashboard.putString("The current towerState is: ", "" + towerState);
          }
          else if (analogSwitchC.getVoltage() >= /*0.700*/0.800 && analogSwitchC.getVoltage() < /*1.400*/1.500)  {
        	  towerState = TOWER_POS_3;
              SmartDashboard.putNumber("Get voltage SwitchC Pos3", analogSwitchC.getVoltage());
              SmartDashboard.putString("The current towerState is: ", "" + towerState);
            }
        
        
        
    } // End autonomousInit

    /**
     * This function is called periodically during autonomous
     */
    
    /*
    public void autonomousPeriodic() {
     // Positions  	
   	 switch(positionState) {
	   case AUTON_POS_1:  
		   switchAPosition[0] = true;
		   SmartDashboard.putString("This should run lowbar: ", "" + positionState);
		   break;   
	   case AUTON_POS_2:  
		   switchAPosition[1] = true;
		   SmartDashboard.putString("The current Case positionState is: ", "" + positionState);			  
		   break;		   
	   case AUTON_POS_3:  
		   switchAPosition[2] = true;
		   SmartDashboard.putString("The current  Case positionState is: ", "" + positionState);		
		   break;	   
	   case AUTON_POS_4:  
		   switchAPosition[3] = true;
		   SmartDashboard.putString("The current  Case  positionState is: ", "" + positionState);		
		   break;	   
	   case AUTON_POS_5:  
		   switchAPosition[4] = true;
		   SmartDashboard.putString("The current  Case  positionState is: ", "" + positionState);			
		   break;	      
	   case AUTON_POS_6:
		   switchAPosition[5] = true;
	       break;
	   case AUTON_POS_7:
		   switchAPosition[6] = true;
		   break;
	   case AUTON_POS_8:
		   switchAPosition[7] = true;
	   default:
		   switchAPosition[0] = true;
		   SmartDashboard.putString("The current  Case  positionState is not supported: ", "" + positionState);
		   break;
	  
   	 } // End Switch 1 Autonomous Positions
   	 
   	 // Defenses 
   	 switch(defenseState){
   	   case DEFENSE_1:
   		   lowBar.startTask();
   		   SmartDashboard.putString("The current defense is lowbar: ","" + defenseState);
   		   break;  	 
   	   case DEFENSE_2:
     	   SmartDashboard.putString("The current defense is roughTerrain", "" + defenseState);
     	   roughTerrain.startTask();
     	   break; 	 
   	   case DEFENSE_3:
   		   SmartDashboard.putString("The current defense is sallyPort: ", "" + defenseState);
   		   sallyPort.startTask();
   		   break;
   	   case DEFENSE_4:
   		   SmartDashboard.putString("The current defense is ramParts: ", "" + defenseState);
   		   ramParts.startTask();
   		   break;   	 
   	   case DEFENSE_5:
   		   SmartDashboard.putString("The current defense is rockWall: ", "" + defenseState); 
   		   rockWall.startTask();
   		   break;   	 
   	   case DEFENSE_6:
   		   SmartDashboard.putString("The current defense is moat: ", "" + defenseState);   	  
   		   moat.startTask();
   		   break;   	 
   	   case DEFENSE_7:
   		   SmartDashboard.putString("The current defense is unknown: ", "" + defenseState);   	  
   		   break;   	 
   	   case DEFENSE_8:
   		   SmartDashboard.putString("The current defense is unknown: ", "" + defenseState);   	   
   		   break;    	 
   	   default:
		   SmartDashboard.putString("This is the default lowbar: ", "" + defenseState);
		   autoTasks.lowBar();
		   break;	   
   	 } //end switch 2 Defenses 
   	 
   	 
     // Tower
   	 switch(towerState){
   	   case TOWER_POS_1: 		 
   		   SmartDashboard.putString("The current  Case  Tower Position 1 is: ", "" + towerState);
   		   break; 	 
   	   case TOWER_POS_2:  		 
      	   SmartDashboard.putString("The current  Case  Tower Position 2 is: ", "" + towerState);
      	   break;     	 
   	   case TOWER_POS_3: 		 
     	   SmartDashboard.putString("The current  Case  Tower Position 3 is: ", "" + towerState);
     	   break; 	 
   	   default:
		   SmartDashboard.putString("This is the default towerState: ", "" + towerState);
		   break;		   
	  } //end switch 3 Tower
 	 SmartDashboard.putString("AUTONOMOUS ENCODER", "" +Math.abs(TPARobotDrive.frontRightMotor.getPosition()));
    } // End AutonomousPeriodic
    
    */

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    // comment 4/18/2016	
    //	robotDrive.arcadeDrive_throttle(); 
    //	loader.loadBallManual(TPARobotMap.loaderMotorSpeed);
   // 	loader.loadBallAuto(TPARobotMap.shooterOptimalSpeed);
   // 	shooter.shootBallManual(TPARobotMap.shooterOptimalSpeed);
   // 	shooter.shootBallAuto(TPARobotMap.shooterOptimalSpeed);
   // 	lifter.runLifterUpManual();
   // 	lifter.runLifterDownManual();
  //  	lifter.autoLiftSustained();
   // 	lifter.autoLiftClick();
   // 	light.turnOnLight();
   // end comment 4/18/2016
    	//try{
//    	navxTeleopAssist.restartNav();
//    	navxTeleopAssist.zeroRobot();
//    	navxTeleopAssist.turnLeft();
//    	navxTeleopAssist.turnRight();
    	/*}catch(NullPointerException e){
    		DriverStation.reportError("COULD NOT FIND NAVX, SHUTTING DOWN ALL PROCESSES DEALING WITH IT",true);
    	}*/
    	SmartDashboard.putString("front: ", ""+frontLiftSwitch.get());
    	SmartDashboard.putString("back: ", ""+backLiftSwitch.get());
    	SmartDashboard.putString("ENCODER ROTATION", ""+TPARobotDrive.frontRightMotor.getPosition());
    	SmartDashboard.putString("CALCULATED ENCODER INCHES: ", ""+calculateDistance2(TPARobotDrive.frontRightMotor.getPosition()));
    	
    	//SmartDashboard.putString("Angle: ", ""+ahrs.getAngle());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
    public double calculateDistance2(double inches){
   	 // bot travels 23.5 inches per 1 wheel revolution, 1440 encoder ticks per 1 wheel revolution
   	 // 1440 encoder ticks divided by wheel circumf in inches 23.5
 	     double encoderTicksPerInch = 1440/23.5;
   	 double targetEncValue =  (inches * encoderTicksPerInch);
   	 
   	 return targetEncValue;
   }  // End calculateDistance
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//1424 units


package frc.robot;
//import frc.lib.util.PID; //IMPLEMENT THIS AT SOMEPOINT...

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drive.DriveSubsystem;
import frc.robot.drive.PIDDriveCommand;
import frc.robot.drive.TalonSRXPIDCommand;
import frc.lib.util.PID;
import frc.robot.drive.DriveStraightCommand;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static DriveSubsystem driveSys;
  private int encoderRaw;
  private int encoderPrev;
  private int encoderDifference;
  private double revMinActual;
  private PID drivePID;
  private double output;
  private static int TOLERANCE;
  private static double SETPOINT;
  private double prevOutput;
  //private static final double kP = 0.000004, kI = 0.0000000002, kD = 0.00004, tolerance = 10;
  private static final double kP = 0.00001, kI = 0.0000, kD = 0.000, tolerance = 10;
//limit to how close to tolerance we can reach


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    driveSys = new DriveSubsystem();
    drivePID = new PID(kP, kI, kD, tolerance);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  // Called periodically regardless of the game period
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }


 /* Teleop Period */
  // Called at the beginning of the Teleop period
  @Override
  public void teleopInit() {
    // Robot.driveSys.resetEncoder();
    // encoderPrev = 0;
    // prevOutput = 0.00;
    // Robot.driveSys.setSpeed(prevOutput);
    // SETPOINT = 1440; //RPM
    
    // drivePID.setSetpoint(SETPOINT);

    Robot.driveSys.resetEncoder();
    encoderPrev = 0;
    prevOutput = 0.00;
    Robot.driveSys.setSpeed(prevOutput);
    SETPOINT = 1440; //RPM
    
    drivePID.setSetpoint(SETPOINT);
    
    
  }
  
  // Called periodically during the Teleop period
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    encoderRaw = Robot.driveSys.getWheelEncoder(); //counts ticks travelled
    drivePID.update(encoderRaw); 
    Robot.driveSys.setSpeed(drivePID.getOutput()); 
    
  //   encoderRaw = Robot.driveSys.getWheelEncoder(); //counts ticks travelled
  //   prevOutput = Robot.driveSys.getOutput();
  //   encoderDifference = encoderRaw - encoderPrev;
  //   revMinActual = encoderDifference*(60000.0/1440);
  //   System.out.println(revMinActual);
  //   drivePID.update(revMinActual, prevOutput);
  //   encoderPrev = encoderRaw;
  //   System.out.println("OUTPUT: " + prevOutput);
  //   prevOutput = drivePID.getOutput(); 
  //   Robot.driveSys.setSpeed(prevOutput);
  // 
  }



  @Override
  public void testInit() {
    
  }
  @Override
  public void testPeriodic() {

  }

}




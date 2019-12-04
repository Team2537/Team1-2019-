package frc.robot.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.PID;
import frc.robot.Robot;
//EDIT!!!!!!!!!!! into other CLASS PID

public class PIDDriveCommand extends Command {
  private int encoderRaw;
  private int encoderPrev;
  private int encoderDifference;
  private double revMinActual;
  private PID drivePID;
  private double output;
  private static int TOLERANCE;
  private static double SETPOINT;
  private double prevOutput;
  private static final double kP = 0.0000000000001, kI = 0.00000, kD = 0.00000, tolerance = 200; //oh...so you had to guess...
  //private static final double TOLERANCE = 5; not used for forward feed

  public PIDDriveCommand() {
    requires(Robot.driveSys);
    drivePID = new PID(kP, kI, kD, tolerance);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    encoderPrev = 0;
    prevOutput = 0.1;
    Robot.driveSys.setSpeed(prevOutput);
    SETPOINT = 500; //RPM
    
    drivePID.setSetpoint(SETPOINT);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    //sets setpoint
    

    encoderRaw = Robot.driveSys.getWheelEncoder(); //counts ticks travelled
    prevOutput = Robot.driveSys.getOutput();
    encoderDifference = encoderRaw - encoderPrev;
    revMinActual = encoderDifference*(60000.0/1440);
    drivePID.update(revMinActual, prevOutput);
    System.out.println(revMinActual);
    encoderPrev = encoderRaw;
    System.out.println(drivePID.getOutput());
    prevOutput = drivePID.getOutput();
    Robot.driveSys.setSpeed(prevOutput);
  }

  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveSys.setSpeed(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {

  }
}
package frc.robot.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.PID;
import frc.robot.Robot;
//EDIT!!!!!!!!!!! into other CLASS PID

public class TalonSRXPIDCommand extends Command {
  private int ENCODERRAW;
  private int ENCODERPREV;
  private int ENCODERDIFFERENCE;
  private double REVMINACTUAL;
  private double ENCODERSPEED;
  private PID drivePID;
  private static final double kP = 0.001, kI = 0.00001, kD = 0.00008; //oh...so you had to guess...
  //private static final double TOLERANCE = 5; not used for forward feed

  public TalonSRXPIDCommand() {
    requires(Robot.driveSys);
    drivePID = new PID(kP, kI, kD);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.driveSys.setSpeed(0);
    Robot.driveSys.setSpeed(.3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
        
        //System.out.println(Robot.driveSys.getSpeed());
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
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveStraightCommand extends Command {
  double leftEncoderStartValue;
  double rightEncoderStartValue;
  
  public DriveStraightCommand() {
    requires(Robot.driveSys);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("WE ROLLIN");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("WE ROLLIN");
    Robot.driveSys.setSpeed(.4);
    System.out.println("ENCODER " + Robot.driveSys.getWheelEncoder());
    
    //double speed = Robot.driveSys.getSpeed()*(1.0/100.0)*(1.0/1424.0)*(1000.0)*(60.0);
    //System.out.println("Speed " + speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
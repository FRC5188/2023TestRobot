package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdDriveDistance extends CommandBase {
  private Drive _driveSubsystem;
  private double _distance;
  private double _maxSpeed;
  private double _minSpeed;
  private boolean _resetEncoders;

  public CmdDriveDistance(Drive driveSubsystem, double distance, double maxSpeed, double minSpeed, boolean resetEncoders) {
    _driveSubsystem = driveSubsystem;
    _distance = distance;
    _maxSpeed = maxSpeed;
    _minSpeed = minSpeed;
    _resetEncoders = resetEncoders;
    
    addRequirements(driveSubsystem);
  }

  /**
   * Initialize the PID controller
   */
  @Override
  public void initialize() {
    _driveSubsystem.drivePIDinit(_distance, _maxSpeed, _minSpeed, _resetEncoders);
  }

  /**
   * Executes the PID
   */
  @Override
  public void execute() {
    _driveSubsystem.executeDrivePID();
  }

  /**
   * Stops the motor
   */
  @Override
  public void end(boolean interrupted) {
    _driveSubsystem.arcadeDrive(0, 0);
  }

  /**
   * Stops the PID when the robot is where it needs to be
   */
  @Override
  public boolean isFinished() {
    return _driveSubsystem.isDriveAtSetPoint();
  }
}

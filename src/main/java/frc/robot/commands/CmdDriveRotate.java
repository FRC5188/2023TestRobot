
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdDriveRotate extends CommandBase {
  private Drive _driveSubsystem;
  private double _heading;
  private double _maxSpeed;
  private double _minSpeed;
  private boolean _resetGyro;

  public CmdDriveRotate(Drive driveSubsystem, double heading, double maxSpeed, double minSpeed, boolean resetGyro) {
    _driveSubsystem = driveSubsystem;
    _heading = heading;
    _maxSpeed = maxSpeed;
    _minSpeed = minSpeed;
    _resetGyro = resetGyro;

    addRequirements(driveSubsystem);
  }

  /**
   * Initializes PID controller
   */
  @Override
  public void initialize() {
    _driveSubsystem.rotatePIDinit(_heading, _maxSpeed, _minSpeed, _resetGyro);
  }

  /**
   * Runs the PID controller
   */
  @Override
  public void execute() {
    _driveSubsystem.executeRotatePID();
  }

  /**
   * Stops the motor
   */
  @Override
  public void end(boolean interrupted) {
    _driveSubsystem.arcadeDrive(0, 0);
  }

  /**
   * Asks if we are at set point
   */
  @Override
  public boolean isFinished() {
    return _driveSubsystem.isRotateAtSetPoint();
  }
}

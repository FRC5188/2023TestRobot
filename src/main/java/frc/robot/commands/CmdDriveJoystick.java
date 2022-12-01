package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class CmdDriveJoystick extends CommandBase {
  /** Creates a new CmdDriveJoystick. */
  private Drive _driveSubsystem;
  private DoubleSupplier _throttle;
  private DoubleSupplier _rotate;

  public CmdDriveJoystick(Drive driveSubsystem, DoubleSupplier throttle, DoubleSupplier rotate) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveSubsystem = driveSubsystem;
    _throttle = throttle;
    _rotate = rotate;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("throttle: " + _throttle.getAsDouble() + " rotate: " + _rotate.getAsDouble());
    _driveSubsystem.arcadeDrive(_throttle.getAsDouble(), _rotate.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveSubsystem.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

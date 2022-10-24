// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CmdDriveJoystick;
import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive _driveSubsystem = new Drive();

  private XboxController _driveController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    _driveSubsystem.setDefaultCommand(new CmdDriveJoystick(_driveSubsystem,
        () -> applyDeadband(_driveController.getLeftY(), 0.1),
        () -> applyDeadband(_driveController.getRightX(), 0.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  private double applyDeadband(double joystickValue, double deadband) {
    double modified = 0;
    deadband = Math.abs(deadband);
    if (joystickValue < -deadband) {
      modified = (joystickValue + 1) / (1 - deadband) - 1;
    } else if (joystickValue > deadband) {
      modified = (joystickValue - 1) / (1 - deadband) + 1;
    }
    System.out.println("Applied deadband: " + modified);
    return modified;
    
  }
}

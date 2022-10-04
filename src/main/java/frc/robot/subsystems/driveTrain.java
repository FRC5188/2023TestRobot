// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveTrain extends SubsystemBase {

  // Declaring our four motors for the wheels
  WPI_TalonFX leftPrimary;
  WPI_TalonFX leftSecondary;
  WPI_TalonFX rightPrimary;
  WPI_TalonFX rightSecondary;

  /** Creates a new driveTrain. */
  public driveTrain() {
    // Initializing our motors
    leftPrimary = new WPI_TalonFX(0);
    leftSecondary = new WPI_TalonFX(0);
    rightPrimary = new WPI_TalonFX(0);
    rightSecondary = new WPI_TalonFX(0);

    // Making the back motors follow the front motors
    leftSecondary.follow(leftPrimary);
    rightSecondary.follow(rightPrimary);

    // Inverting the right motors so robot doesn't spin when setting motors to the same value
    rightPrimary.setInverted(true);
    rightSecondary.setInverted(true);

    leftPrimary.setNeutralMode(NeutralMode.Brake);
    leftSecondary.setNeutralMode(NeutralMode.Brake);
    rightPrimary.setNeutralMode(NeutralMode.Brake);
    rightSecondary.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setleftpower(double power){

    
  }



}

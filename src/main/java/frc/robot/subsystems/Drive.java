// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    // Declaring our four motors for the wheels
    WPI_TalonFX leftPrimary;
    WPI_TalonFX leftSecondary;
    WPI_TalonFX rightPrimary;
    WPI_TalonFX rightSecondary;

    /** Creates a new driveTrain. */
    public Drive() {
        // Initializing our motors
        leftPrimary = new WPI_TalonFX(1);
        leftSecondary = new WPI_TalonFX(2);
        rightPrimary = new WPI_TalonFX(3);
        rightSecondary = new WPI_TalonFX(4);

        // Making the back motors follow the front motors
        leftSecondary.follow(leftPrimary);
        rightSecondary.follow(rightPrimary);

        // Inverting the right motors so robot doesn't spin when setting motors to the
        // same value
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

    public void setMotorPower(double leftMotorPower, double rightMotorPower) {
        // sets motor power , input of two doubles
        rightPrimary.set(rightMotorPower);
        leftPrimary.set(leftMotorPower);
    }

    /**
     * Takes throttle and rotation, converts into motor power, sets motors
     * 
     * @param throttle The value for moving forward/backward
     * @param rotation The value for rotating cw and ccw
     */
    public void arcadeDrive(double throttle, double rotation) {
        double rDrive;
        double lDrive;
        // when going backwards, reverse rotation
        if (throttle < -Constants.ARCADE_DRIVE_DEADBAND) {
            rotation *= -1;
        }

        //
        if (Math.abs(throttle) < Constants.QUICK_TURN_DEADBAND) {
            lDrive = rotation * Constants.QUICK_TURN_MULTIPLIER;
            rDrive = -rotation * Constants.QUICK_TURN_MULTIPLIER;
        } else {

            lDrive = throttle * (1 + Math.min(0, rotation));
            rDrive = throttle * (1 - Math.max(0, rotation));

        }
        setMotorPower(lDrive, rDrive);
    }
}

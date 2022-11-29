// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * @file Drive.java
 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 */
public class Drive extends SubsystemBase {

    // Declaring our four motors for the wheels
    private WPI_TalonFX _leftPrimary;
    private WPI_TalonFX _leftSecondary;
    private WPI_TalonFX _rightPrimary;
    private WPI_TalonFX _rightSecondary;

    // Declaring our gyro.
    private AHRS _gyro;

    // PID Controller for Drive & Rotate
    private PIDController _drivePID;
    private PIDController _rotatePID;

    // Also associated with PID Controller, and represents the maximum and minimum speeds for the controllers. 
    private double _driveMaxSpeed;
    private double _driveMinSpeed;
    private double _rotateMaxSpeed;
    private double _rotateMinSpeed;

    /** Creates a new driveTrain. */
    public Drive() {
        // Initializing our motors
        _leftPrimary = new WPI_TalonFX(1);
        _leftSecondary = new WPI_TalonFX(2);
        _rightPrimary = new WPI_TalonFX(3);
        _rightSecondary = new WPI_TalonFX(4);

        // Use the integrated device to determine postion of the motor. Setting up
        // encoders.
        _leftPrimary.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _leftSecondary.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _rightPrimary.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        _rightSecondary.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // Initializing PID Controllers
        // Mind these are currently hard coded, change later.
        _drivePID = new PIDController(0, 0, 0);
        _rotatePID = new PIDController(0, 0, 0);

        // Initializing Gyro
        _gyro = new AHRS();

        // Making the back motors follow the front motors
        _leftSecondary.follow(_leftPrimary);
        _rightSecondary.follow(_rightPrimary);

        // Inverting the right motors so robot doesn't spin when setting motors to the
        // same value
        _rightPrimary.setInverted(true);
        _rightSecondary.setInverted(true);

        _leftPrimary.setNeutralMode(NeutralMode.Brake);
        _leftSecondary.setNeutralMode(NeutralMode.Brake);
        _rightPrimary.setNeutralMode(NeutralMode.Brake);
        _rightSecondary.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    public void setMotorPower(double leftMotorPower, double rightMotorPower) {
        // sets motor power , input of two doubles
        _rightPrimary.set(rightMotorPower);
        _leftPrimary.set(leftMotorPower);
    }

    /**
     * Takes throttle and rotation, converts into motor power, sets motors
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

    /**
     * Sets the encoder postion to zero, resetting the encoder position.
     */
    public void resetEncoders() {
        _leftPrimary.setSelectedSensorPosition(0);
        _rightPrimary.setSelectedSensorPosition(0);
    }

    public void resetGyro() {
        _gyro.reset();
    }

    /**
     * Initializes the PID for Drive.
     * 
     * @param distance
     * @param maxSpeed
     * @param minSpeed
     * @param resetEncoders
     */
    public void drivePIDinit(double distance, double maxSpeed, double minSpeed, boolean resetEncoders) {
        // Check to reset if yes then reset.
        if (resetEncoders) {
            resetEncoders();
        }

        // Tells the PID controller where you want to run to. Calculates how to get
        // where we want to go.
        _drivePID.setSetpoint(distance);

        // Store max and min.
        _driveMaxSpeed = maxSpeed;
        _driveMinSpeed = minSpeed;
    }

    /**
     * Initializes the PID for rotate.
     * 
     * @param distance
     * @param maxSpeed
     * @param minSpeed
     * @param resetEncoders
     */
    public void rotatePIDinit(double distance, double maxSpeed, double minSpeed, boolean resetEncoders) {
        // Check to reset if yes then reset.
        if (resetEncoders) {
            resetEncoders();
        }

        // Tells the PID controller where you want to run to. Calculates how to get
        // where we want to go.
        _rotatePID.setSetpoint(distance);

        // Store max and min.
        _rotateMaxSpeed = maxSpeed;
        _rotateMinSpeed = minSpeed;
    }

    /**
     * Finds the current postion of the PID Controller in inches.
     * 
     * @return Current postion in inches.
     */
    public double getPosition() {
        final int ENCODER_TICS_PER_ROTATION = 2048;
        final double WHEEL_DIAMETER = 6.0;
        double encoderTics;

        encoderTics = (_leftPrimary.getSelectedSensorPosition() + _rightPrimary.getSelectedSensorPosition()) / 2;
        return (encoderTics / ENCODER_TICS_PER_ROTATION) * (Math.PI * WHEEL_DIAMETER);

    }

    /**
     * Finds the currents angle from the gyro.
     * 
     * @return Angle represented as a double.
     */
    public double getAngle() {
        return _gyro.getAngle();
    }

    /**
     * Executes of Driver PID through setting the power of primary motors torward
     * power based on calculated distance.
     */
    public void executeDrivePID() {
        double power = _drivePID.calculate(getPosition()) * _driveMaxSpeed;

        // if the power is less than in speed set to min speed
        if (power < _driveMinSpeed) {
            power = _driveMaxSpeed;
        }

        _leftPrimary.set(power);
        _rightPrimary.set(power);
    }

    /**
     * Executes of Rotate PID through setting the power of primary motors torward
     * power based on calculated distance.
     */
    public void executeRotatePID() {
        double power = _rotatePID.calculate(getAngle()) * _rotateMaxSpeed;

        if (power < _rotateMinSpeed) {
            power = _rotateMinSpeed;
        }

        _leftPrimary.set(power);
        _rightPrimary.set(-power);
    }

    /**
     * Asks if the Driver PID is at the point it needs to go.
     * 
     * @return Are we there yet?
     */
    public boolean isDriveAtSetPoint() {
        return _drivePID.atSetpoint();
    }

    /**
     * Asks if the Rotate PID is at the point it needs to go.
     * 
     * @return Are we there yet?
     */
    public boolean isRotateAtSetPoint() {
        return _rotatePID.atSetpoint();
    }

}

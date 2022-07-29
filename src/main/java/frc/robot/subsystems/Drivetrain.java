// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MathUtils;

public class Drivetrain extends SubsystemBase {

  private TalonFX leftFront, leftBack, rightFront, rightBack;
  private TalonFX[] arr;

  private AHRS gyro;

  // Auton Stuff that you'll need:
  private Pose2d pose;
  private DifferentialDriveOdometry odometry;
  private Field2d field;
  private SimpleMotorFeedforward feedforward;
  private DifferentialDriveKinematics kinematics;
  private PIDController leftController, rightController;
  private DifferentialDrive differentialDrive;

  /** Constructor */
  public Drivetrain() {
    leftFront = new TalonFX(0);
    leftBack = new TalonFX(1);
    rightFront = new TalonFX(2);
    rightBack = new TalonFX(3);

    arr = new TalonFX[] { leftFront, leftBack, rightFront, rightBack };

    for (TalonFX motor : arr)
      motor.configFactoryDefault();

    for (TalonFX motor : arr)
      motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDIdx,
          Constants.kTimeoutMs);

    rightFront.setInverted(TalonFXInvertType.Clockwise);
    rightBack.setInverted(TalonFXInvertType.Clockwise);
    leftFront.setInverted(TalonFXInvertType.CounterClockwise);
    leftBack.setInverted(TalonFXInvertType.CounterClockwise);

    for (TalonFX motor : arr)
      motor.configSupplyCurrentLimit(Constants.kCurrentLimit, Constants.kTimeoutMs);

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();

    // initialize all your Auton thingz below:
    // ____
  }

  /**
   * Method for regular joystick driving during teleop through implementation of
   * the cheesyIshDrive, and then applies the speed to the motors
   * 
   * @param throttle  (double) power towards forward and backward movement
   * @param wheel     (double) power towards turning movement
   * @param quickTurn (boolean) status of quickTurnButton
   */
  public void cheesyIshDrive(double throttle, double wheel, boolean quickTurn) {
    throttle = MathUtils.handleDeadband(throttle, Constants.Drivetrain.kThrottleDeadband);
    wheel = MathUtils.handleDeadband(wheel, Constants.Drivetrain.kWheelDeadband);

    if (quickTurn)
      wheel *= 0.8;

    double left = 0, right = 0;
    // setDrivetrainMotorSpeed((throttle+wheel) / 2, (throttle-wheel) / 2);
    final double denominator = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * Constants.Drivetrain.kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= Constants.Drivetrain.kWheelGain;
    Twist2d motion = new Twist2d(throttle, 0, wheel);
    if (Math.abs(motion.dtheta) < 1E-9) {
      left = motion.dx;
      right = motion.dx;
    } else {
      double delta_v = Constants.Drivetrain.kTrackWidthInches * motion.dtheta
          / (2 * Constants.Drivetrain.kTrackScrubFactor);
      left = motion.dx + delta_v;
      right = motion.dx - delta_v;
    }

    double scaling_factor = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));

    setDrivetrainMotorSpeed(left / scaling_factor, right / scaling_factor);
  }

  @Override
  public void periodic() {
    // Update Stuff
    log();
  }

  public void log() {
    // SmartDashboard .put__() go here
  }

  /**
   * Sets a voltage percentage output to each motor given values for left and
   * right motors
   * 
   * @param left  the percentage [-1, 1] for the left motors
   * @param right the percentage [-1, 1] from the right motors
   */
  public void setDrivetrainMotorSpeed(double left, double right) {
    leftFront.set(ControlMode.PercentOutput, left);
    leftBack.set(ControlMode.PercentOutput, left);
    rightFront.set(ControlMode.PercentOutput, right);
    rightBack.set(ControlMode.PercentOutput, right);
  }

  /**
   * Returns the rotation as a Rotation2d object
   * 
   * @return Rotation2d of current rotation
   */
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  /**
   * Returns the distance traveled by the two motor parameters.
   * Precondition: motors are on the same side
   * 
   * @param m1 motor
   * @param m2 other motor
   * @return distance traveled in meters
   */
  public double getDistanceTravelled(TalonFX m1, TalonFX m2) {
    double ticks = (m1.getSelectedSensorPosition() + m2.getSelectedSensorPosition()) / 2.0;

    // returns the converted values
    return MathUtils.convertTicksToMeters(
        ticks,
        Constants.Drivetrain.kTicksPerRevolution,
        Constants.Drivetrain.kGearRatio,
        Constants.Drivetrain.kwheelCircumference);
  }

  /**
   * Resets the encoders
   */
  public void resetEncoders() {
    for (TalonFX motor : arr)
      motor.setSelectedSensorPosition(0);
  }

  /**
   * All of your Auton Getters and Setters go below, 
   * including getRamseteCommand(Trajectory tr), which is the holy 
   * mother (or grandmother, even) of auton
   */
}

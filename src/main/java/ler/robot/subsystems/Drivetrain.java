/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import ler.robot.RobotMap;

import ler.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  double maxOutput=1;
  
  private boolean isInverted = false;

  public final PIDController pidController = new PIDController(7, 0.018, 1.5);

  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(RobotMap.leftMotors, RobotMap.rightMotors);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;
  /**
   * Creates a new DriveSubsystem.
   */
  public Drivetrain() {
    // Sets the distance per pulse for the encoders
    
    /*
    RobotMap.leftEncoders.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    RobotMap.rightEncoders.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    */

    RobotMap.leftDriveSpark1.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    RobotMap.leftDriveSpark2.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    RobotMap.leftDriveSpark3.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);

    RobotMap.rightDriveSpark1.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    RobotMap.rightDriveSpark2.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    RobotMap.rightDriveSpark3.getEncoder().setPositionConversionFactor(DriveConstants.kEncoderDistancePerPulse);
    
    
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftAverageEncoderPosition(),
      getRightAverageEncoderPosition());
  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftAverageEncoderVelocity(), getRightAverageEncoderVelocity());
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param left the commanded forward movement
   * @param right the commanded rotation
   */
  public void tankDrive(double left, double right) {
    //System.out.println("L Input: " + left + " R Input: " + right);

    if (Math.abs(left)<DriveConstants.DEADZONE){
      left = 0;

    }
    if (Math.abs(right)<DriveConstants.DEADZONE){
      right = 0;
    }
    //Use to check Deadzone input/output
    //System.out.println("L Deadzoned: " + left + " R Deadzoned: " + right);


    //Slow it down
    left *= 0.6;
    right *= 0.6;
    //System.out.println(left);
    //System.out.println(right);
    
    //Use Math.min to apply max speed rules, then
    //Multiply      the magnitude              by the direction
    // left = Math.min(Math.abs(left), maxOutput)*Math.signum(left);
    // right = Math.min(Math.abs(right), maxOutput)*Math.signum(right);

    //System.out.println("L:"+left+"\tR:"+right);

    if(isInverted){
      double tempRight = left * -1;
      left = right * -1;
      right = tempRight;
    }
    RobotMap.leftMotors.set(left);
    RobotMap.rightMotors.set(right);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    RobotMap.leftMotors.setVoltage(leftVolts);
    RobotMap.rightMotors.setVoltage(-rightVolts);
    drive.feed();
  }

  public void invertControls(){
    isInverted = !isInverted;
  }

  public void resetEncoders() {
    RobotMap.leftDriveSpark1.getEncoder().setPosition(0);
    RobotMap.leftDriveSpark2.getEncoder().setPosition(0);
    RobotMap.leftDriveSpark3.getEncoder().setPosition(0);

    RobotMap.rightDriveSpark1.getEncoder().setPosition(0);
    RobotMap.rightDriveSpark2.getEncoder().setPosition(0);
    RobotMap.rightDriveSpark3.getEncoder().setPosition(0);
  }

  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }

   /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    RobotMap.gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(RobotMap.gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

    /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return RobotMap.gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  //get average encoder positions
  private double getLeftAverageEncoderPosition() {
    return (
      RobotMap.leftDriveSpark1.getEncoder().getPosition() +
      RobotMap.leftDriveSpark2.getEncoder().getPosition() +
      RobotMap.leftDriveSpark3.getEncoder().getPosition()) / 3;
  }

  private double getRightAverageEncoderPosition() {
    return (
      RobotMap.rightDriveSpark1.getEncoder().getPosition() +
      RobotMap.rightDriveSpark2.getEncoder().getPosition() +
      RobotMap.rightDriveSpark3.getEncoder().getPosition()) / 3;
  }

  //get average encoder velocities
  private double getLeftAverageEncoderVelocity() {
    return (
      RobotMap.leftDriveSpark1.getEncoder().getVelocity() +
      RobotMap.leftDriveSpark2.getEncoder().getVelocity() +
      RobotMap.leftDriveSpark3.getEncoder().getVelocity()) / 3;
  }

  private double getRightAverageEncoderVelocity() {
    return (
      RobotMap.rightDriveSpark1.getEncoder().getVelocity() +
      RobotMap.rightDriveSpark2.getEncoder().getVelocity() +
      RobotMap.rightDriveSpark3.getEncoder().getVelocity()) / 3;
  }

}

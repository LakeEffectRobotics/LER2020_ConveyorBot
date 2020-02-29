/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

import ler.robot.subsystems.Shooter;

import ler.robot.Constants.Mappings;
import ler.robot.Constants.ShooterConstants;
import ler.robot.Constants.DriveConstants;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {
  // The motors on the left side of the drive.
  public static final CANSparkMax leftDriveSpark1 = new CANSparkMax(DriveConstants.LEFT_DRIVE_SPARK_1, MotorType.kBrushless);
  public static final CANSparkMax leftDriveSpark2 = new CANSparkMax(DriveConstants.LEFT_DRIVE_SPARK_2, MotorType.kBrushless);
  public static final CANSparkMax leftDriveSpark3 = new CANSparkMax(DriveConstants.LEFT_DRIVE_SPARK_3, MotorType.kBrushless);
  

  // The motors on the right side of the drive.
  public static final CANSparkMax rightDriveSpark1 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_SPARK_1, MotorType.kBrushless);
  public static final CANSparkMax rightDriveSpark2 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_SPARK_2, MotorType.kBrushless);
  public static final CANSparkMax rightDriveSpark3 = new CANSparkMax(DriveConstants.RIGHT_DRIVE_SPARK_3, MotorType.kBrushless);

  //make Speedcontroller groups
  //TODO: organise this maybe?
  public static final SpeedControllerGroup leftMotors = 
    new SpeedControllerGroup(leftDriveSpark1, leftDriveSpark2, leftDriveSpark3);

  public static final SpeedControllerGroup rightMotors = 
    new SpeedControllerGroup(rightDriveSpark1, rightDriveSpark2, rightDriveSpark3);

  //encoders
  //TODO: these might not be neccesary, since our motors use neos which already have encoders. So I might be able to remove these later
  // The left-side drive encoder
  public static final Encoder leftEncoders =
      new Encoder(
        DriveConstants.kLeftEncoderPorts[0], 
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderPorts[2],
        DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  public static final Encoder rightEncoders =
      new Encoder(
        DriveConstants.kRightEncoderPorts[0], 
        DriveConstants.kRightEncoderPorts[1], 
        DriveConstants.kRightEncoderPorts[2],
        DriveConstants.kRightEncoderReversed);

  // The talons on the shooter
  public static final CANSparkMax shooterTopSpark = new CANSparkMax(Mappings.SHOOTER_TOP_SPARK, MotorType.kBrushless);
  public static final CANSparkMax shooterBottomSpark = new CANSparkMax(Mappings.SHOOTER_BOTTOM_SPARK, MotorType.kBrushless);

  //The conveyor 
  public static final TalonSRX conveyorMotor = new TalonSRX(Mappings.CONVEYOR_TALON);
  //The intake
  public static final TalonSRX intakeRoller = new TalonSRX(Mappings.INTAKE_TALON);

  //Gyro
  public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();


  public static void init(){
    leftDriveSpark2.follow(leftDriveSpark1);
    leftDriveSpark3.follow(leftDriveSpark1);

    leftDriveSpark1.setInverted(true);
    rightDriveSpark1.setInverted(false);

    leftDriveSpark1.setOpenLoopRampRate(0.5);
    rightDriveSpark1.setOpenLoopRampRate(0.5);

    rightDriveSpark2.follow(rightDriveSpark1);
    rightDriveSpark3.follow(rightDriveSpark1);

    //shooter init
    shooterTopSpark.getPIDController().setP(ShooterConstants.kP);
    shooterTopSpark.getPIDController().setI(ShooterConstants.kI);
    shooterTopSpark.getPIDController().setD(ShooterConstants.kD);
    shooterTopSpark.getPIDController().setFF(ShooterConstants.kF);

    shooterBottomSpark.getPIDController().setP(ShooterConstants.kP);
    shooterBottomSpark.getPIDController().setI(ShooterConstants.kI);
    shooterBottomSpark.getPIDController().setD(ShooterConstants.kD);
    shooterBottomSpark.getPIDController().setFF(ShooterConstants.kF);
  }
}

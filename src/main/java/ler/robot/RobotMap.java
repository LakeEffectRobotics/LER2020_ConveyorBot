/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import ler.robot.subsystems.Shooter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.</p>
 */
public final class RobotMap {
  public static boolean xboxDrive = false;
  static final double CONVERSION_FACTOR = 0.6;
  static final double RAMP_RATE = 0.2;

  /**
   * Constants used for CAN loop elements.
   */
  public static final class CANConstants {

    public static final int LEFT_DRIVE_SPARK_1 = 2;
    public static final int LEFT_DRIVE_SPARK_2 = 3;
    public static final int LEFT_DRIVE_SPARK_3 = 4;
    public static final int RIGHT_DRIVE_SPARK_1 = 5;
    public static final int RIGHT_DRIVE_SPARK_2 = 6;
    public static final int RIGHT_DRIVE_SPARK_3 = 7;
    
    public static final int INTAKE_TALON = 8;

    public static final int SHOOTER_BOTTOM_LEFT_SPARK = 9;
    public static final int SHOOTER_TOP_LEFT_SPARK = 10;
    public static final int SHOOTER_BOTTOM_RIGHT_SPARK = 11;
    public static final int SHOOTER_TOP_RIGHT_SPARK = 12;

    public static final int CLIMBER_TALON = 13;

    public static final int CONVEYOR_TALON = 14;
    public static final int ANGLE_ELEVATION = 15;
  
  }

  /**
   * Constants used for solenoids attached to the PCM.
   */
  public static final class SOLENOIDConstants {
    
    public static final int CONVEYOR_DOWN = 0;
    public static final int CONVEYOR_UP = 1;

    public static final int INTAKE_DOWN = 2;
    public static final int INTAKE_UP = 3;

    public static final int CLIMBER_DOWN = 4;
    public static final int CLIMBER_UP = 5;
  }

  /**
   * Constants used to map OI controls.
   */
  public static final class OIConstants {
    public static final int DRIVER_XBOX_CONTROLLER = 0;

    public static final int LEFT_DRIVER_JOYSTICK = 0;
    public static final int RIGHT_DRIVER_JOYSTICK = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;

    public static final int HALF_SPEED_BUTTON = 2;
    public static final int INVERT_CONTROLS_BUTTON = 1;
    public static final int LIMELIGHT_AIM_BUTTON = 2;

    //TODO: change these button mappings as well
    public static final int XBOX_HALF_SPEED_BUTTON = Button.kX.value;
    public static final int XBOX_INVERT_CONTROLS_BUTTON = Button.kX.value;
    public static final int XBOX_LIMELIGHT_AIM_BUTTON = Button.kX.value;

    public static final int INTAKE_BUTTON = Button.kB.value;
    //TODO: change buttons to correct buttons
    public static final int INTAKE_EXTEND_BUTTON = Button.kX.value;

    public static final int CONVEYOR_PISTON_BUTTON = Button.kX.value;

    public static final int SHOOTER_CONTROL_BUTTON = Button.kBumperRight.value;
    public static final int SHOOT_BUTTON = Button.kBumperLeft.value; 
    //needs to be changed
    //we should think about how this one works
    public static final int SHOOTER_LONGBOMB_ANGLE_BUTTON = Button.kX.value; 
    public static final int SHOOTER_LIMELIGHT_ANGLE_BUTTON = Button.kX.value;

    //TODO: change these as well
    public static final int CLIMBER_EXTEND_BUTTON = Button.kX.value;
    public static final int CLIMBER_WINCH_BUTTON = Button.kX.value;

    public static final int GYRO_DRIVE_BUTTON = Button.kX.value;
    public static final int REVERSE_BOTH_BUTTON = Button.kX.value;
    public static final int REVERSE_INTAKE_BUTTON = Button.kX.value;
    public static final int CONVEYOR_BUTTON = Button.kX.value;

  }


  /* 
    CHECKSTYLE OFF: ConstantNameCheck
    Motor controllers are an exception to naming conventions, so checkstyle must be disabled for this block
  */
  // The motors on the left side of the drive.
  public static final CANSparkMax leftDriveSpark1 = new CANSparkMax(CANConstants.LEFT_DRIVE_SPARK_1, MotorType.kBrushless);
  public static final CANSparkMax leftDriveSpark2 = new CANSparkMax(CANConstants.LEFT_DRIVE_SPARK_2, MotorType.kBrushless);
  public static final CANSparkMax leftDriveSpark3 = new CANSparkMax(CANConstants.LEFT_DRIVE_SPARK_3, MotorType.kBrushless);

  // The motors on the right side of the drive.
  public static final CANSparkMax rightDriveSpark1 = new CANSparkMax(CANConstants.RIGHT_DRIVE_SPARK_1, MotorType.kBrushless);
  public static final CANSparkMax rightDriveSpark2 = new CANSparkMax(CANConstants.RIGHT_DRIVE_SPARK_2, MotorType.kBrushless);
  public static final CANSparkMax rightDriveSpark3 = new CANSparkMax(CANConstants.RIGHT_DRIVE_SPARK_3, MotorType.kBrushless);

  // The talons on the shooter
  public static final CANSparkMax shooterTopLeftSpark = new CANSparkMax(CANConstants.SHOOTER_TOP_LEFT_SPARK, MotorType.kBrushless);
  public static final CANSparkMax shooterBottomLeftSpark = new CANSparkMax(CANConstants.SHOOTER_BOTTOM_LEFT_SPARK, MotorType.kBrushless);
  public static final CANSparkMax shooterTopRightSpark = new CANSparkMax(CANConstants.SHOOTER_TOP_RIGHT_SPARK, MotorType.kBrushless);
  public static final CANSparkMax shooterBottomRightSpark = new CANSparkMax(CANConstants.SHOOTER_BOTTOM_RIGHT_SPARK, MotorType.kBrushless);

  //The conveyor (conveyor + angle + piston)
  public static final TalonSRX conveyorMotor = new TalonSRX(CANConstants.CONVEYOR_TALON);
  public static final CANSparkMax angleElevation = new CANSparkMax(CANConstants.ANGLE_ELEVATION, MotorType.kBrushless);
  public static final DoubleSolenoid conveyorValve = new DoubleSolenoid(SOLENOIDConstants.CONVEYOR_DOWN, SOLENOIDConstants.CONVEYOR_UP);

  //The intake (talon + piston)
  public static final TalonSRX intakeRoller = new TalonSRX(CANConstants.INTAKE_TALON);
  public static final DoubleSolenoid intakeArm = new DoubleSolenoid(SOLENOIDConstants.INTAKE_DOWN, SOLENOIDConstants.INTAKE_UP);

  

  // Climber control
  public static final TalonFX climberFalcon = new TalonFX(CANConstants.CLIMBER_TALON);
  public static final DoubleSolenoid climberPiston = new DoubleSolenoid(SOLENOIDConstants.CLIMBER_DOWN, SOLENOIDConstants.CLIMBER_UP);

  // Gyro
  public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  /* CHECKSTYLE ON: ConstantNameCheck */


  /** Initialize robot systems.*/
  public static void init(){
    gyro.calibrate();

    xboxDrive = SmartDashboard.getBoolean("xbox drive", xboxDrive);


    leftDriveSpark1.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);
    leftDriveSpark2.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);
    leftDriveSpark3.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);
    rightDriveSpark1.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);
    rightDriveSpark2.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);
    rightDriveSpark3.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR*Math.PI);

    // leftDriveSpark1.getEncoder().

    leftDriveSpark2.follow(leftDriveSpark1);
    leftDriveSpark3.follow(leftDriveSpark1);

    leftDriveSpark1.setInverted(false);
    rightDriveSpark1.setInverted(true);

    leftDriveSpark1.setOpenLoopRampRate(RAMP_RATE);
    rightDriveSpark1.setOpenLoopRampRate(RAMP_RATE);

    rightDriveSpark2.follow(rightDriveSpark1);
    rightDriveSpark3.follow(rightDriveSpark1);

    //SHOOTER PIDs
    shooterTopLeftSpark.getPIDController().setP(Shooter.kP);
    shooterTopLeftSpark.getPIDController().setI(Shooter.kI);
    shooterTopLeftSpark.getPIDController().setD(Shooter.kD);
    shooterTopLeftSpark.getPIDController().setFF(Shooter.kF);

    shooterTopRightSpark.getPIDController().setP(Shooter.kP);
    shooterTopRightSpark.getPIDController().setI(Shooter.kI);
    shooterTopRightSpark.getPIDController().setD(Shooter.kD);
    shooterTopRightSpark.getPIDController().setFF(Shooter.kF);

    shooterBottomLeftSpark.getPIDController().setP(Shooter.kP);
    shooterBottomLeftSpark.getPIDController().setI(Shooter.kI);
    shooterBottomLeftSpark.getPIDController().setD(Shooter.kD);
    shooterBottomLeftSpark.getPIDController().setFF(Shooter.kF);

    shooterBottomRightSpark.getPIDController().setP(Shooter.kP);
    shooterBottomRightSpark.getPIDController().setI(Shooter.kI);
    shooterBottomRightSpark.getPIDController().setD(Shooter.kD);
    shooterBottomRightSpark.getPIDController().setFF(Shooter.kF);
  }
}

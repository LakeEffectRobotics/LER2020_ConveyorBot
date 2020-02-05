/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class RobotMap {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kLeftMotor3Port = 3;
    public static final int kRightMotor1Port = 4;
    public static final int kRightMotor2Port = 5;
    public static final int kRightMotor3Port = 6;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 1;
  }

  // The motors on the left side of the drive.
  public static final CANSparkMax m_leftMotors = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  public static final CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  public static final CANSparkMax m_leftMotor3 = new CANSparkMax(DriveConstants.kLeftMotor3Port, MotorType.kBrushless);
  

  // The motors on the right side of the drive.
  public static final CANSparkMax m_rightMotors = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  public static final CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
  public static final CANSparkMax m_rightMotor3 = new CANSparkMax(DriveConstants.kRightMotor3Port, MotorType.kBrushless);

  // The robot's drive
  public static final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public static void init(){
    m_leftMotor2.follow(m_leftMotors);
    m_leftMotor3.follow(m_leftMotors);

    m_leftMotors.setInverted(true);
    m_rightMotors.setInverted(false);

    m_rightMotor2.follow(m_rightMotors);
    m_rightMotor3.follow(m_rightMotors);
  }
}

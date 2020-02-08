/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ler.robot.RobotMap;

public class Drivetrain extends SubsystemBase {
  
  double maxOutput=1;

  /*
  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
                  DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
                  DriveConstants.kRightEncoderReversed);
  */

  /**
   * Creates a new DriveSubsystem.
   */
  public Drivetrain() {
    // Sets the distance per pulse for the encoders
    /*
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    */
  }

  

  /**
   * Drives the robot using arcade controls.
   *
   * @param left the commanded forward movement
   * @param right the commanded rotation
   */
  public void tankDrive(double left, double right) {
    //slow it down
    left *= 0.25;
    right *= 0.25;
    //System.out.println(left);
    //System.out.println(right);
    
    //Use Math.min to apply max speed rules, then
    //Multiply      the magnitude              by the direction
    // left = Math.min(Math.abs(left), maxOutput)*Math.signum(left);
    // right = Math.min(Math.abs(right), maxOutput)*Math.signum(right);

    //System.out.println("L:"+left+"\tR:"+right);

    RobotMap.leftDriveSpark1.set(left);
    RobotMap.rightDriveSpark1.set(right);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  /*
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  */

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */

   /*
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }
  */

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */

   /*
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }
  */

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */

   /*
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }
  */

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }
}

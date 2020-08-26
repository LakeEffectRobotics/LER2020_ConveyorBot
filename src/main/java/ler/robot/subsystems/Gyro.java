/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ler.robot.RobotMap;

/**
 * Subsystem to represent the gyroscope.
 */
public class Gyro extends SubsystemBase {

  double setpoint = 0;

  ADXRS450_Gyro gyro;

  /**
   * Creates a new Gyro.
   */
  public Gyro() {

  }
  
  /**
   * Initialize the gyro. This should be called once in robot initialisation
   */
  public void init(){
    gyro.calibrate();
  }

  /**
   * Make getAngle return 0 for current heading.
   */
  public void zero(){
    setpoint = gyro.getAngle();
  }

  /**
   * Get the drivetrain output required to hold heading.
   * @param l The target left output
   * @param r The target right output
   * @return A double[] with the required {leftOutput, rightOutput}
   */
  public double[] getStraightOutput(double l, double r) {
    final double ANGLE_TOLERANCE = 1;
    final double kP = 0.02;
    double lOut = l;
    double rOut = r;
    double currentAngle = getAngle();
    
    
    if(Math.abs(currentAngle) > ANGLE_TOLERANCE){	// adjust values to compensate for turning drift	
      double modifier = currentAngle * kP;	// make amount of correction proportional to angle
      lOut += modifier;
      rOut -= modifier;
    }
    
    lOut = lOut > 1 ? 1 : lOut;	// ensure values are within -1 to 1 range
    lOut = lOut < -1 ? -1 : lOut;
    rOut = rOut > 1 ? 1 : rOut;
    rOut = rOut < -1 ? -1 : rOut;
    
    return new double[] {lOut, rOut};
  }

  /**
   * Get the current heading of the robot.
   * @return Current heading, relative to init or last zero
   */
  public double getAngle(){
    return setpoint - gyro.getAngle();
  }

  /**
   * Get the current heading of the robot, relative to init.
   * @return Current heading, relative to init
   */
  public double getAbsoluteAngle() {
    return -gyro.getAngle();
  }
}

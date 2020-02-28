/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import ler.robot.RobotMap;

public class Shooter extends SubsystemBase{
  
  public static final double kP = 0.000065;
  public static final double kI = 0.0;
  public static final double kD = 0.000002;
  public static final double kF = 0.000090;

  private static final double SPIN_CONSTANT = 0.8;

  public static final double[] SPEEDS = {0, 0.8};
  public static final int ZEROSPEED = 0;
  public static final double limelightSpeedScaling = 0.01;
  public static final double SHOOTER_TARGET_SPEED = 7900;
  public static final double SHOOTER_TOP_TARGET_SPEED = SHOOTER_TARGET_SPEED;
  public static final double SHOOTER_BOTTOM_TARGET_SPEED = -SHOOTER_TARGET_SPEED*SPIN_CONSTANT;

  public long spoolTime = System.currentTimeMillis();

  //Cycles per revolution of encoders on shooter
  public static final int cPR = 64;
  /**
   * Creates a new ShooterSubsystem.
   */
  public Shooter() {

  }

  public void setShooterSpeed(int speedArrayPosition){
    setSpecificShooterSpeed(SPEEDS[speedArrayPosition]);
  }

  public void setSpecificShooterSpeed(double speed){
    spoolTime = System.currentTimeMillis()+100;
    if(speed == 0) {
      RobotMap.shooterTopLeftSpark.set(0);
      RobotMap.shooterBottomLeftSpark.set(0);
      RobotMap.shooterTopRightSpark.set(0);
      RobotMap.shooterBottomRightSpark.set(0);
    }
    else {
      RobotMap.shooterTopLeftSpark.getPIDController().setReference((speed), ControlType.kVelocity);
      RobotMap.shooterTopLeftSpark.getPIDController().setReference((speed), ControlType.kVelocity);
      RobotMap.shooterBottomLeftSpark.getPIDController().setReference(-speed*SPIN_CONSTANT, ControlType.kVelocity);
      RobotMap.shooterBottomRightSpark.getPIDController().setReference(-speed*SPIN_CONSTANT, ControlType.kVelocity);
    }
  }

  public double getTopLeftSparkSpeed(){
    return(RobotMap.shooterTopLeftSpark.getEncoder().getVelocity());
  }
  
  public double getTopRightSparkSpeed(){
    return(RobotMap.shooterTopRightSpark.getEncoder().getVelocity());
  }

  public double getBottomLeftSparkSpeed(){
    return(RobotMap.shooterBottomLeftSpark.getEncoder().getVelocity());
  }
  
  public double getBottomRightSparkSpeed(){
    return(RobotMap.shooterBottomRightSpark.getEncoder().getVelocity());
  }

  //speed should be how far the bot can shoot straight up
  //should be used with getSpeed() from limelight
  public double getVelocityFromLimelight(double speed){
    return(speed * limelightSpeedScaling);
  }

}

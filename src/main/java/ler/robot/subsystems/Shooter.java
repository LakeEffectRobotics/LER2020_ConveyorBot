/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import ler.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Shooter extends SubsystemBase{
  private int currentSpeed = 0;
  /**
   * Creates a new ShooterSubsystem.
   */
  public Shooter() {

  }

  public void setShooterSpeed(int speed){
    setSpecificShooterSpeed(RobotMap.ShooterConstants.speeds[speed]);
  }

  public void setSpecificShooterSpeed(double speed){
    //talonShooterTop.set(ControlMode.PercentOutput, speeds[currentSpeed]);
    RobotMap.talonShooterTop.set(ControlMode.Velocity, speed);
    //talonShooterBottom.set(ControlMode.PercentOutput, -speeds[currentSpeed]);
    RobotMap.talonShooterBottom.set(ControlMode.Velocity, -speed);
  }

  public void talonResetPos(){
    RobotMap.talonShooterTop.setSelectedSensorPosition(0);
    RobotMap.talonShooterBottom.setSelectedSensorPosition(0);
  }

  public double getRevolutionsPerSecond(int rpsinput){
    return ((rpsinput/RobotMap.ShooterConstants.cPR)*10)*0.314;
  }
}

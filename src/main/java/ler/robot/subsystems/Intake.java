/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import ler.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Intake extends SubsystemBase {
  
  
  public static final double ROLLER_SPEED = 0.80;

  /**
   * Creates a new Intake.
   */
  public Intake() {

  }

  public void startIntake(double speed){
    RobotMap.intakeRoller.set(ControlMode.PercentOutput, speed );

  }
  public void stopIntake(){
    RobotMap.intakeRoller.set(ControlMode.PercentOutput, 0);

  }

  public void extendIntake(){
    RobotMap.intakeArm.set(Value.kForward);
  }

  public void retractIntake(){
    RobotMap.intakeArm.set(Value.kReverse);
  }

  public boolean isExtended(){
    if(RobotMap.intakeArm.get() == Value.kForward){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

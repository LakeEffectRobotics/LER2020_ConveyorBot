/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Subsystem representing the intake arm and rollers.
 */
public class Intake extends SubsystemBase {
  
  
  public static final double ROLLER_SPEED = 0.80;

  TalonSRX roller;
  DoubleSolenoid arm;
  
  /**
   * Creates a new Intake.
   * 
   * @param roller The motor driving the intake roller
   * @param arm The DoubleSolenoid controlling arm deployment
   */
  public Intake(TalonSRX roller, DoubleSolenoid arm) {
    this.roller = roller;
    this.arm = arm;
  }

  /**
   * Drive the intake roller.
   *
   * @param speed The % to drive at
   */
  public void startIntake(double speed){
    roller.set(ControlMode.PercentOutput, speed);

  }

  /**
   * Stop the intake roller.
   */
  public void stopIntake(){
    roller.set(ControlMode.PercentOutput, 0);

  }

  /**
   * Deploy intake.
   */
  public void extendIntake(){
    arm.set(Value.kForward);
  }

  /**
   * Retract intake.
   */
  public void retractIntake(){
    arm.set(Value.kReverse);
  }

  /**
   * Get if the intake is deployed.
   * 
   * @return <code>true</code> if the intake is deployed
   */
  public boolean isExtended(){
    if(arm.get() == Value.kForward){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

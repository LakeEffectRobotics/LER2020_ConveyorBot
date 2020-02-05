/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import ler.robot.subsystems.Drivetrain;

public class HalveDriveSpeed extends CommandBase {
  private final Drivetrain drivetrain;

  public HalveDriveSpeed(Drivetrain drive) {
    drivetrain = drive;
  }

  @Override
  public void initialize() {
    drivetrain.setMaxOutput(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setMaxOutput(1);
  }
}

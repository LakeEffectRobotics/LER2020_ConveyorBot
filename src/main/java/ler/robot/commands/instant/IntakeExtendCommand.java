/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.instant;

import ler.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Extend the intake.
 */
public class IntakeExtendCommand extends InstantCommand {
  Intake intake;

  /**
   * Extend the intake.
   * @param intake Intake subsystem
   */
  public IntakeExtendCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.isExtended()){
      intake.retractIntake();
    } else {
      intake.extendIntake();
    }
  }
}

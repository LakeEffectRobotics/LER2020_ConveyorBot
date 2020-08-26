/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.instant;

import ler.robot.subsystems.Conveyor;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Command to toggle conveyor cover.
 */
public class ConveyorExtendCommand extends InstantCommand {
  Conveyor conveyor;

  /**
   * Toggle the conveyor cover.
   * @param conveyor Conveyor subsystem
   */
  public ConveyorExtendCommand(Conveyor conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setValveState(!conveyor.isExtended());
  }
}

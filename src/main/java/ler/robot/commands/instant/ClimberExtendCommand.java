/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.instant;

import ler.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Toggle the climber.
 */
public class ClimberExtendCommand extends InstantCommand {
  Climber climber;

  /**
   * Toggle climber.
   * @param climber Climber subsystem
   */
  public ClimberExtendCommand(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(climber.isElevatorExtended()){
      climber.lowerElevator();
    } else {
      climber.raiseElevator();
    }
  }
}

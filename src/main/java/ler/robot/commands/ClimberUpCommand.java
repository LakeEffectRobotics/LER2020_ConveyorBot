/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Climber;

/**
 * Extend climber.
 */
public class ClimberUpCommand extends CommandBase {
  Climber climber;

  /**
   * Creates a new ClimberUpCommand.
   * 
   * @param climber The climber subsystem
   */
  public ClimberUpCommand(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.raiseElevator();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

}

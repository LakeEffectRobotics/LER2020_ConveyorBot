/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Intake;

/**
 * Command to invert both intake and conveyor.
 */
public class InverseBothCommand extends CommandBase {
  Intake intake;
  Conveyor conveyor;

  /**
   * Creates a new InverseBothCommand.
   * 
   * @param intake Intake object
   * @param conveyor Conveyor object
   */
  public InverseBothCommand(Intake intake, Conveyor conveyor) {
    this.intake = intake;
    this.conveyor = conveyor;
    
    addRequirements(intake);
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Drive the intake and conveyor (INVERSE)
    intake.startIntake(-Intake.ROLLER_SPEED);
    conveyor.setConveyorSpeed(-Conveyor.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

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

public class IntakeCommand extends CommandBase {
  Intake intake;
  Conveyor conveyor;

  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(Intake intake, Conveyor conveyor) {
    this.intake = intake;
    this.conveyor = conveyor;
    
    addRequirements(intake);
    addRequirements(conveyor);
    System.out.println("Constructing");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Executing");
    //Drive the intake and conveyor
    intake.extendIntake();
    intake.startIntake(-Intake.ROLLER_SPEED);
    conveyor.setConveyorSpeed(Conveyor.INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    //conveyor.StopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

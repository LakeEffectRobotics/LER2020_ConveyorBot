/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Intake;
import ler.robot.RobotMap;
import ler.robot.subsystems.Conveyor;

public class IntakeCommand extends CommandBase {
  /**
   * Creates a new IntakeCommand.
   */
  private final Intake intake;
  private final Conveyor conveyor;
  public IntakeCommand(Intake sub1, Conveyor sub2) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = sub1;
    conveyor = sub2;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.StartIntake(RobotMap.IntakeConstants.intakeSpeed);
    conveyor.SetConveyorSpeed(RobotMap.ConveyorConstants.ConveyorIntakeSpeed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Drivetrain;
import ler.robot.subsystems.Limelight;

public class LimelightAimCommand extends CommandBase {
  private Drivetrain drivetrain;
  private Limelight limelight;
  /**
   * Creates a new LimelightAimCommand.
   */
  public LimelightAimCommand(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.pidController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutput = drivetrain.pidController.calculate(limelight.getX());

    //aim bot
    drivetrain.tankDrive(-pidOutput/15, pidOutput/15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(limelight.getX()) < 1){
      return true;
    }
    return false;
  }
}
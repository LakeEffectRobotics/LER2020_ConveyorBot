/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Limelight;
import ler.robot.subsystems.Shooter;

/**
 * Start the shooter.
 */
public class ShooterStartCommand extends CommandBase {
  private Shooter shooter;
  private Limelight limelight;


  /**
   * Creates a new ShooterStartCommand.
   * @param shooter Shooter subsystem
   * @param limelight Limelight subsystem
   */
  public ShooterStartCommand(Shooter shooter, Limelight limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
    addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //shooter.setSpecificShootersSpeed(shooter.getVelocityFromLimelight(limelight.getSpeed()));
    shooter.setSpecificShootersSpeed(Shooter.SHOOTER_TARGET_SPEED);
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

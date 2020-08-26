/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Shooter;

/**
 * Stop the shooter wheels.
 */
public class ShooterStopCommand extends CommandBase {
  private Shooter shooter;
  private Conveyor conveyor;

  /**
   * Creates a new ShooterStopCommand.
   * @param shooter Shooter subsystem
   * @param conveyor Conveyor subsytem
   */
  public ShooterStopCommand(Shooter shooter, Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.conveyor = conveyor;
    addRequirements(shooter);
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpecificShootersSpeed(0);

    conveyor.setConveyorSpeed(0);
    /*
    RobotMap.shooterTopTalon.set(ControlMode.PercentOutput, 0);
    RobotMap.shooterBottomTalon.set(ControlMode.PercentOutput, 0);
    */
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

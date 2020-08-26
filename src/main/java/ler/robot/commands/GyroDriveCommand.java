/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.Robot;
import ler.robot.Tools;
import ler.robot.subsystems.Drivetrain;
import ler.robot.subsystems.Gyro;

/**
 * Command to activate gyro-controlled driving.
 */
public class GyroDriveCommand extends CommandBase {
  
  Drivetrain drivetrain;
  Gyro gyro;

  /**
   * Creates a new GyroDriveCommand.
   * 
   * @param d The drivetrain subsystem
   * @param g The gyro subsystem
   */
  public GyroDriveCommand(Drivetrain d, Gyro g) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = d;
    gyro = g;
    addRequirements(drivetrain);
    addRequirements(gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.zero();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double l = Tools.getAdaptedSpeed(Robot.oi.leftDriverJoystick.getY());
    double r = Tools.getAdaptedSpeed(Robot.oi.rightDriverJoystick.getY());
    	
    l = Math.abs(l) > Math.abs(r) ? l : r;
    r = l;
    	
    double[] straightGyroOutput = gyro.getStraightOutput(l, r);
    
    drivetrain.tankDrive(straightGyroOutput[0], straightGyroOutput[1]);
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

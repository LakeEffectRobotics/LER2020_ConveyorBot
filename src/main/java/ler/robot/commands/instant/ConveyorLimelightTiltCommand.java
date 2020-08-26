/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.instant;

import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * Control tilt with limelight.
 */
public class ConveyorLimelightTiltCommand extends InstantCommand {
  Conveyor conveyor;
  Limelight limelight;

  /**
   * Control the tlit.
   * 
   * @param conveyor Conveyor subsystem
   * @param limelight Limelight subsystem
   */
  public ConveyorLimelightTiltCommand(Conveyor conveyor, Limelight limelight) {
    this.conveyor = conveyor;
    this.limelight = limelight;
    //this addrequirements might make it so that you can't run the conveyor
    addRequirements(conveyor);
    addRequirements(limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setSpecificConveyorAngle(limelight.getAngle());
  }
}

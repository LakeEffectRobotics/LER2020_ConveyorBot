/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import ler.robot.RobotContainer;

/**
 * Command used to test new wpilib features
 */
public class TestAutoCommand extends SequentialCommandGroup {

  /**
   * Creates a new TestAutoCommand.
   * 
   * @param container The {@link RobotContainer} used to get subsystems
   */
  public TestAutoCommand(RobotContainer container) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super();
    // System.out.println("Initialising");
    //addCommands(new AutoDriveStraightCommand(container.drivetrain, container.gyro, 3000, 0.30, -12*15));
    //Add command to drive 60 inches at 25% power
    addCommands(new AutoDriveStraightCommand(container.drivetrain, container.gyro, 3000, 0.25, 10));
    // addCommands(new GyroTurnCommand(container.drivetrain, container.gyro, 90, false));
  }
}

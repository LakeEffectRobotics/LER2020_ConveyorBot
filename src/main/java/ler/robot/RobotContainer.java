/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import ler.robot.RobotMap;
import ler.robot.commands.DefaultDrive;
import ler.robot.commands.DefaultShooter;
import ler.robot.commands.HalveDriveSpeed;
import ler.robot.subsystems.Drivetrain;
import ler.robot.subsystems.Shooter;

import static edu.wpi.first.wpilibj.XboxController.Button;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Shooter shooter = new Shooter();

  private final Drivetrain drivetrain = new Drivetrain();

  // The autonomous routines

  // A simple auto routine that drives forward a specified distance, and then stops.
  /*
  private final Command m_simpleAuto =
      new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                        m_robotDrive);
  */

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  //private final Command m_complexAuto = new ComplexAuto(m_robotDrive, m_hatchSubsystem);

  // A chooser for autonomous commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    Robot.oi.init(drivetrain);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive

    //default shooter commands
    shooter.setDefaultCommand(
      new DefaultShooter(
          shooter,
          () -> Robot.oi.driverController.getXButton(),
          () -> Robot.oi.driverController.getAButtonReleased(),
          () -> Robot.oi.driverController.getYButtonReleased()));
    drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
            drivetrain,
            () -> Robot.oi.driverController.getY(GenericHID.Hand.kLeft),
            () -> Robot.oi.driverController.getY(GenericHID.Hand.kRight)));

    // Add commands to the autonomous command chooser
    //m_chooser.addOption("Simple Auto", m_simpleAuto);
    //m_chooser.addOption("Complex Auto", m_complexAuto);

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

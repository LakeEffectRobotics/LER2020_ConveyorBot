/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import ler.robot.commands.DefaultDrive;
import ler.robot.commands.autonomous.AutoCommand;
//import ler.robot.commands.IntakeCommand;
import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Drivetrain;
import ler.robot.subsystems.Intake;
import ler.robot.subsystems.Shooter;
import ler.robot.subsystems.Limelight;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  final Shooter shooter = new Shooter();
  final Intake intake = new Intake();
  final Conveyor conveyor = new Conveyor();
  final Drivetrain drivetrain = new Drivetrain();
  final Limelight limelight = new Limelight();


  // The autonomous routines

  // A simple auto routine that drives forward a specified distance, and then stops.
  /*
  private final Command m_simpleAuto =
      new DriveDistance(AutoConstants.kAutoDriveDistanceInches, AutoConstants.kAutoDriveSpeed,
                        m_robotDrive);
  */

  // A complex auto routine that drives forward, drops a hatch, and then drives backward.
  private final Command complexAuto = new AutoCommand();

  // A chooser for autonomous commands
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    Robot.oi.init(this);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive

    drivetrain.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDrive(
            drivetrain,
            () -> Robot.oi.leftDriverJoystick.getY(),
            () -> Robot.oi.rightDriverJoystick.getY()));

    /*intake.setDefaultCommand(
      new DefaultIntake(
            intake,
            conveyor,
            () -> Robot.oi.operatorController.getTriggerAxis(GenericHID.Hand.kRight)));
    */
    // Add commands to the autonomous command chooser
    //m_chooser.addOption("Simple Auto", m_simpleAuto);
    autoChooser.addOption("Complex Auto", complexAuto);

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

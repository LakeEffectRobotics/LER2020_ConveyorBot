/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import ler.robot.commands.DriveCommand;
//import ler.robot.commands.IntakeCommand;
import ler.robot.subsystems.Conveyor;
import ler.robot.subsystems.Gyro;
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
  final Gyro gyro = new Gyro();
  final Intake intake = new Intake();
  final Conveyor conveyor = new Conveyor();
  final Drivetrain drivetrain = new Drivetrain();
  final Limelight limelight = new Limelight();

  final UsbCamera webcam;

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
    Robot.oi.init(this);

    webcam = CameraServer.getInstance().startAutomaticCapture(0);
    webcam.setResolution(320, 240);
    webcam.setFPS(20);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive

    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));

    /*intake.setDefaultCommand(
      new DefaultIntake(
            intake,
            conveyor,
            () -> Robot.oi.operatorController.getTriggerAxis(GenericHID.Hand.kRight)));
    */
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.RamseteCommand;

import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;

import ler.robot.subsystems.Drivetrain;

import ler.robot.Constants.AutoConstants;
import ler.robot.Constants.DriveConstants;

import java.util.List;

public class AutoCommand extends CommandBase {
  /**
   * Creates a new AutoCommand.
   */
  //TODO: might have to pass that into the command
  private final Drivetrain robotDrive = new Drivetrain();

  //TODO: this all probably has to be moved
  //From https://github.com/wpilibsuite/allwpilib/blob/master/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/ramsetecommand/RobotContainer.java

  // Create a voltage constraint to ensure we don't accelerate too fast
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                      DriveConstants.kvVoltSecondsPerMeter,
                                      DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          10);

  TrajectoryConfig config =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      config
  );

  RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      robotDrive::getPose,
      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                  DriveConstants.kvVoltSecondsPerMeter,
                                  DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      robotDrive::getWheelSpeeds,
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      new PIDController(DriveConstants.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      robotDrive::tankDriveVolts,
      robotDrive
  );

  public AutoCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Run path following command, then stop at the end.
    //TODO: where should this go
    ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

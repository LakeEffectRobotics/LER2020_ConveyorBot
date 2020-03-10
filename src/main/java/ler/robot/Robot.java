/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;


import javax.rmi.ssl.SslRMIClientSocketFactory;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import ler.robot.vision.GripPipeline;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import ler.robot.commands.autonomous.AutoDriveStraightCommand;
import ler.robot.commands.autonomous.TestAutoCommand;
import ler.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  public static OI oi = new OI();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    RobotMap.init();

    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  
    SmartDashboard.putNumber("Gyro Angle", RobotMap.gyro.getAngle());
    System.out.println(robotContainer.drivetrain.getLeftEncoder()+","+robotContainer.drivetrain.getRightEncoder());
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //autonomousCommand = robotContainer.getAutonomousCommand();
    //Create a new auto commandGroup

    System.out.println("Running autonomous");
    autonomousCommand = new TestAutoCommand(robotContainer);// AutoDriveStraightCommand(robotContainer.drivetrain, robotContainer.gyro, 3000, 0.25, 24);

    // new TestAutoCommand(robotContainer);
    // new AutoDriveStraightCommand(robotContainer.drivetrain, robotContainer.gyro, 3000, 0.5, 5);
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    // if (autonomousCommand != null) {
      System.out.println("Starting Scheduler");
      autonomousCommand.schedule();
    // }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println("AUTO");

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("ShooterTopRightVel", RobotMap.shooterTopRightSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("ShooterBottomRightVel", RobotMap.shooterBottomRightSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("ShooterTopLeftVel", RobotMap.shooterTopLeftSpark.getEncoder().getVelocity());
    SmartDashboard.putNumber("ShooterBottomLeftVel", RobotMap.shooterBottomLeftSpark.getEncoder().getVelocity());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

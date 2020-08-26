/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import ler.robot.Robot;
import ler.robot.RobotMap;
import ler.robot.Tools;
import ler.robot.subsystems.Drivetrain;
//import ler.robot.subsystems.Gyro;

/**
 * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
 * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.RunCommand}.
 */
public class DriveCommand extends CommandBase {
  private final Drivetrain drivetrain;
  //private final Gyro gyro;

  static final double DEADBAND = 0.025;
  static final double STRAIGHT_DIFF = 0.25;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   */
  public DriveCommand(Drivetrain subsystem) {
    drivetrain = subsystem;
    addRequirements(drivetrain);

  }

  @Override
  public void execute() {
    //System.out.println("Driving");
    
    //Using the logistic function for adapted speed
    double left;
    double right;
    if(RobotMap.xboxDrive){
      left = Robot.oi.driverXboxController.getY(Hand.kLeft);
      right = Robot.oi.driverXboxController.getY(Hand.kRight);
    } else {
      left = Robot.oi.leftDriverJoystick.getY();
      right = Robot.oi.rightDriverJoystick.getY();
    }

    double leftSpeed = Tools.getAdaptedSpeed(left);
    double rightSpeed = Tools.getAdaptedSpeed(right);
    
    double average = (leftSpeed + rightSpeed)/2;

    // if sticks are close and speed reasonable, go straight
    if(Math.abs(leftSpeed - rightSpeed) < DEADBAND && Math.abs(average) > STRAIGHT_DIFF){
      leftSpeed = average;
      rightSpeed = average;
    }

    //drivetrain.tankDrive(left.getAsDouble(), rightSpeed.getAsDouble());
    drivetrain.tankDrive(leftSpeed, rightSpeed);
    }
}

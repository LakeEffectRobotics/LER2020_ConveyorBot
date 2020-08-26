/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO: Properly document various functions
/**
 * Subsystem to represent the Limelight camera.
 */
public class Limelight extends SubsystemBase {
  private NetworkTable table;
  private NetworkTable table3d;

  /**
   * Creates a new Limelight.
   */
  public Limelight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    table = NetworkTableInstance.getDefault().getTable("limelight");
    table3d = NetworkTableInstance.getDefault().getTable("camtran");
    SmartDashboard.putNumber("Limelight: Dist to side board", getDistance2D());
    SmartDashboard.putNumber("Limelight: Dist to target (3D)", getDistance3D());
    SmartDashboard.putNumber("Angle to target", getAngle());
    SmartDashboard.putNumber("Estimated required shooter velocity", getSpeed());

  }

  public double getX(){
    return(table.getEntry("tx").getDouble(0.0));
  }

  public double getY(){
    return(table.getEntry("ty").getDouble(0.0));
  }

  public double getX3d(){
    return(table3d.getEntry("x").getDouble(0.0));
  }

  public double getY3d(){
    return(table3d.getEntry("y").getDouble(0.0));
  }

  public double getZ3d(){
    return(table3d.getEntry("z").getDouble(0.0));
  }

  // Calculate horizontal distance from (Limelight aperature) to (sideboard under target)
  public double getDistance2D(){
    return(Math.sqrt(Math.pow(getX3d(), 2) + Math.pow(getZ3d(), 2)));
  }

  // Calculate distance from (Limelight aperature() to (center of target)
  public double getDistance3D(){
    return(Math.sqrt(Math.pow(getX3d(), 2) + Math.pow(getY3d(), 2) + Math.pow(getZ3d(), 2)));
  }

  //do a bunch of math to get trajectory
  //the math for these things is kindof at https://www.desmos.com/calculator/8tvpcylqvr
  public double getAngle(){
    return(Math.atan2(-2 * getY3d(), getDistance2D()));
  }

  public double getSpeed(){
    return(Math.sqrt(Math.pow(-1/2 * getDistance2D(), 2) + Math.pow(getY3d(), 2)));
  }
}

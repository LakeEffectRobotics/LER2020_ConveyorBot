/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ler.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    
    public static final class DriveConstants{
        public static final int LEFT_DRIVE_SPARK_1 = 5;
        public static final int LEFT_DRIVE_SPARK_2 = 6;
        public static final int LEFT_DRIVE_SPARK_3 = 7;
        public static final int RIGHT_DRIVE_SPARK_1 = 2;
        public static final int RIGHT_DRIVE_SPARK_2 = 3;
        public static final int RIGHT_DRIVE_SPARK_3 = 4;

        public static final int[] kLeftEncoderPorts = new int[]{0, 1, 2};
        public static final int[] kRightEncoderPorts = new int[]{3, 4, 5};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final boolean kGyroReversed = true;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;

        public static final double DEADZONE = 0.15;
    }

    public static final class ShooterConstants{
        public static final double kP = 0.000065;
        public static final double kI = 0.0;
        public static final double kD = 0.000002;
        public static final double kF = 0.000090;

        public static final double SPIN_CONSTANT = 0.8;

        public static final double[] SPEEDS = {0, 0.8};
        public static final int ZEROSPEED = 0;
        public static final double limelightSpeedScaling = 0.01;
        public static final double SHOOTER_TARGET_SPEED = 7900;
        public static final double SHOOTER_TOP_TARGET_SPEED = SHOOTER_TARGET_SPEED;
        public static final double SHOOTER_BOTTOM_TARGET_SPEED = -SHOOTER_TARGET_SPEED*SPIN_CONSTANT;
    }

    public static final class Mappings {
        public static final int SHOOTER_TOP_SPARK = 9;
        public static final int SHOOTER_BOTTOM_SPARK = 8;
    
        public static final int CONVEYOR_TALON = 12;
    
       public static final int INTAKE_TALON = 11;
      }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class OIConstants{
        public static final int LEFT_DRIVER_JOYSTICK = 0;
        public static final int RIGHT_DRIVER_JOYSTICK = 1;
        public static final int OPERATOR_CONTROLLER_PORT = 2;
        //public static final int DRIVER_CONTROLLER_PORT = 1;
        
        public static final class ButtonMappings {

            public static final int HALF_SPEED_BUTTON = 2;
            public static final int INVERT_CONTROLS_BUTTON = 1;
            public static final int LIMELIGHT_AIM_BUTTON = 2;

            public static final int INTAKE_BUTTON = Button.kB.value;

            public static final int SHOOTER_CONTROL_BUTTON = Button.kBumperRight.value;
            public static final int SHOOT_BUTTON = Button.kBumperLeft.value; 
            public static final int SHOOTER_TILT_BUTTON = Button.kBumperLeft.value; 
        }
    }

    
}

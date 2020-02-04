/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public abstract class Constants {
    public static enum RobotType {
        CompBot,
        PracticeBot,
        KOPChassis,
        TestBoard;
    }
    public static RobotType GenerateConstants(RobotType robot) {
        switch (robot) {
            case CompBot: {

            } break;
            case PracticeBot: {

            } break;
            case KOPChassis: {
                DriveConstants.kHasDriveTrain = true;
                DriveConstants.ksVolts = 0.146;
                DriveConstants.kvVoltSecondsPerMeter = 2.17;
                DriveConstants.kaVoltSecondsSquaredPerMeter = 0.308;
                DriveConstants.kPDriveVel = 2.59;
                DriveConstants.kTrackwidthMeters = 0.66;
                DriveConstants.kDriveKinematics = new DifferentialDriveKinematics(
                    DriveConstants.kTrackwidthMeters);
                DriveConstants.kFeedforward = new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter);
                DriveConstants.kLeftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kRightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kMaxSpeedMetersPerSecond = 3;
                DriveConstants.kMaxAccelerationMetersPerSecondSquared = 3;
                DriveConstants.kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
                DriveConstants.kRamseteB = 2;
                DriveConstants.kRamseteZeta = 0.7;
                DriveConstants.kMaxRPM = 5700;
                DriveConstants.kWheelDiameter = 0.152;
                DriveConstants.kMotorGearsToWheelGears = 8.44;
                DriveConstants.kRevolutionsToMeters = Math.PI * DriveConstants.kWheelDiameter / 8.44;
                DriveConstants.kRPMtoMetersPerSecond = 
                    Math.PI * DriveConstants.kWheelDiameter / (60 * DriveConstants.kMotorGearsToWheelGears);
                DriveConstants.kGyroReversed = true;
                DriveConstants.kLeftMotor1Port = 4;
                DriveConstants.kLeftMotor2Port = 5;
                DriveConstants.kRightMotor1Port = 3;
                DriveConstants.kRightMotor2Port = 6;
            } break;
            case TestBoard: {
                DriveConstants.kHasDriveTrain = false;
            } break;
        }
        return robot;
    }
    public static class DriveConstants {
        public static boolean kHasDriveTrain = false;

        // Feedforward constants
        public static double ksVolts = 0.146;
        public static double kvVoltSecondsPerMeter = 2.17;
        public static double kaVoltSecondsSquaredPerMeter = 0.308;

        // Example value only - as above, this must be tuned for your drive!
        public static double kPDriveVel = 2.59;

        public static double kTrackwidthMeters = 0.66;
        public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);
        
        // Added from WPI's DifferentialDrive voltage system
        public static SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter);
        
        // Added from WPI's DifferentialDrive voltage system
        public static PIDController kLeftPIDController = new PIDController(kPDriveVel, 0, 0);
        public static PIDController kRightPIDController = new PIDController(kPDriveVel, 0, 0);

        public static double kMaxSpeedMetersPerSecond = 3; //Originally 3m/s
        public static double kMaxAccelerationMetersPerSecondSquared = 3; //Originally 3m/s/s
        public static double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; //TEMPLATE VALUE

        // Reasonable baseline values for a RAMSETE follower in units of meters and
        // seconds
        public static double kRamseteB = 2;
        public static double kRamseteZeta = 0.7;

        //Custom constants for NEOs and SparkMax's
        public static double kMaxRPM = 5700;
        public static double kWheelDiameter = 0.152;
        public static double kMotorGearsToWheelGears = 8.44;
        public static double kRevolutionsToMeters = Math.PI * kWheelDiameter / 8.44;
        public static double kRPMtoMetersPerSecond = Math.PI * kWheelDiameter / (60 * kMotorGearsToWheelGears);
        
        public static boolean kGyroReversed = true;

        public static int kLeftMotor1Port = 4;
        public static int kLeftMotor2Port = 5;
        public static int kRightMotor1Port = 3;
        public static int kRightMotor2Port = 6;
    }
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kManipulatorControllerPort = 1;
    }
    
    public static final class AutoConstants {
        public static double kMaxSpeedMetersPerSecond = 3; //Originally 3 m/s/s
        public static double kMaxAccelerationMetersPerSecondSquared = 3; //Originally 3 m/s/s

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public static final class ControlPanelConstants {
        public static boolean kHasControlPanel = false;
        //What I2C port to plug the REV Color Sensor V3 into
        public static I2C.Port kColorSensorPort = I2C.Port.kOnboard;
        public static int kWheelRotatorMotorPort = 7;

        //These values have been calculated by REV Robotics
        public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }
}

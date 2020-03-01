package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
@Deprecated
public final class OldConstants {
    public final static class DriveConstants {
        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or
        // theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining
        // these
        // values for your robot.
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
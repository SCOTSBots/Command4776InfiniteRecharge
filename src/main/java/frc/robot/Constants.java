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
import edu.wpi.first.wpilibj.util.ColorShim;

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
        TestBoard,
        JeffBot;
    }
    public static String RobotName;
    public static RobotType GenerateConstants(RobotType robot) {
        switch (robot) {
            case CompBot: {
                RobotName = "Karen the CompBot";

                DriveConstants.kHasDriveTrain = true;
                DriveConstants.kHasGyro = true;
                DriveConstants.kHasPixy = false;
                ControlPanelConstants.kHasControlPanel = false;
                ClimberConstants.kHasClimber = true;
                IntakeConstants.kHasIntake = true;
                ShooterConstants.kHasShooter = true;
                ShooterConstants.kHasTurret = true;
                LEDConstants.kHasLEDs = false;

                IntakeConstants.kHasIntakeColorSensor = true;

                DriveConstants.kDebug = false;
                ControlPanelConstants.kDebug = false;
                ClimberConstants.kDebug = false;
                IntakeConstants.kDebug = false;
                ShooterConstants.kDebug = false;
                LEDConstants.kDebug = false;

                //CAN WIRING:
                ShooterConstants.kShooterMotor1Port = 31;
                ShooterConstants.kShooterMotor2Port = 32;
                ShooterConstants.kHoodAngleServoPort = 0; //PWM, not CAN
                ShooterConstants.kTurretMotorPort = 21;
                DriveConstants.kLeftMotor1Port = 37;
                DriveConstants.kLeftMotor2Port = 38;
                DriveConstants.kRightMotor1Port = 34;
                DriveConstants.kRightMotor2Port = 30;
                IntakeConstants.kIntakeFlipMotorPort = 22;
                IntakeConstants.kIntakeMotorPort = 35;
                IntakeConstants.kConveyorMotor1Port = 19;
                IntakeConstants.kConveyorMotor2Port = 7;
                ClimberConstants.kClimberMotor1Port = 39;
                ClimberConstants.kClimberMotor2Port = 36;
                ControlPanelConstants.kColorSensorPort = I2C.Port.kOnboard; //I2C, not CAN
                ControlPanelConstants.kWheelRotatorMotorPort = 31;
                ControlPanelConstants.kMechanismRotatorServoPort = 31; //PWM, not CAN
                
                IntakeConstants.kIntakeColorSensor = 0; //Multiplexer I2C, not CAN
                IntakeConstants.kShooterColorSensor = 6; //Multiplexer I2C, not CAN
                ControlPanelConstants.kControlPanelColorSensor = 3; //Multiplexer I2C, not CAN

                //DRIVE CONSTANTS
                DriveConstants.ksVolts = 0.195;
                DriveConstants.kvVoltSecondsPerMeter = 2.78;
                DriveConstants.kaVoltSecondsSquaredPerMeter = 0.381;
                DriveConstants.kPDriveVel = 2.57;
                DriveConstants.kTrackwidthMeters = 3.0;
                DriveConstants.kDriveKinematics = new DifferentialDriveKinematics(
                    DriveConstants.kTrackwidthMeters);
                DriveConstants.kFeedforward = new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter);
                DriveConstants.kLeftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kRightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kMaxSpeedMetersPerSecond = 1.0;
                DriveConstants.kMaxAccelerationMetersPerSecondSquared = 1.0;
                DriveConstants.kMaxAngularSpeedRadiansPerSecond = 0.5 * Math.PI;
                //DriveConstants.kRamseteB = 2;
                //DriveConstants.kRamseteZeta = 0.7;
                DriveConstants.kMaxRPM = 5700;
                DriveConstants.kWheelDiameter = 0.148;
                DriveConstants.kMotorGearsToWheelGears = 10.7;
                DriveConstants.kRevolutionsToMeters = Math.PI * DriveConstants.kWheelDiameter / DriveConstants.kMotorGearsToWheelGears;
                DriveConstants.kRPMtoMetersPerSecond = 
                    Math.PI * DriveConstants.kWheelDiameter / (60 * DriveConstants.kMotorGearsToWheelGears);
                DriveConstants.kGyroReversed = true;

                //INTAKE CONSTANTS
                IntakeConstants.kIntakeFlipP = 0.04;
                IntakeConstants.kIntakeFlipI = 1e-5;
                IntakeConstants.kIntakeFlipD = 0.2;
                IntakeConstants.kIntakeFlipIz = 0;
                IntakeConstants.kIntakeFlipFF = 0;
                IntakeConstants.kIntakeFlipMaxOutput = 1;
                IntakeConstants.kIntakeFlipMinOutput = -1;
                IntakeConstants.kIntakeFlipInEncoder = -5.7467; //NOTE: This is NOT ZERO BECUASE YOU MUST RESET THE ROBOT TO THE OUTERMOST POSITION!
                // IntakeConstants.kIntakeFlipOutEncoder = -38.5; //NOTE: Different sprocket ratio, so different encoder value!
                IntakeConstants.kIntakeFlipOutEncoder = -35.5; //NOTE: Different sprocket ratio, so different encoder value!
                
                IntakeConstants.kIntakeColorThesholdR = 6000; //MAKE SURE THE GRAPH IS ORANGE AND NOT REDb
                IntakeConstants.kIntakeColorThesholdG = 10200; //MAKE SURE THE GRAPH IS RED AND NOT GREEN
                IntakeConstants.kIntakeColorThesholdB = 0;
                IntakeConstants.kIntakeColorThesholdIR = 0;
                IntakeConstants.kIntakeColorThesholdProximity = 200;//full practice 2 says 200?

                IntakeConstants.kShooterColorThesholdR = 0;
                IntakeConstants.kShooterColorThesholdG = 0;
                IntakeConstants.kShooterColorThesholdB = 0;
                IntakeConstants.kShooterColorThesholdIR = 0;
                IntakeConstants.kShooterColorThesholdProximity = 285;

                //SHOOTER CONSTANTS
                
                ShooterConstants.kShooterP = 6e-2; 
                ShooterConstants.kShooterI = 0;
                ShooterConstants.kShooterD = 0; 
                ShooterConstants.kShooterIz = 0; 
                ShooterConstants.kShooterFF = 0.000015; 
                ShooterConstants.kShooterMaxOutput = 1; 
                ShooterConstants.kShooterMinOutput = -1;
                ShooterConstants.kMaxRPM = 5700;

                ShooterConstants.kMaxTurretAngle = 200;//126 = 180degrees
                // ShooterConstants.kMaxTurretAngle = 100;//126 = 180degrees
                ShooterConstants.kMinTurretAngle = -200;//-126 = 180 degrees
                // ShooterConstants.kMinTurretAngle = -100;//-126 = 180 degrees

                LEDConstants.kLEDPWMPort = 9;
            } break;
            case PracticeBot: {
                RobotName = "The Practice Robot";

                DriveConstants.kHasDriveTrain = true;
                DriveConstants.kHasGyro = true;
                DriveConstants.kHasPixy = false;
                ControlPanelConstants.kHasControlPanel = false;
                ClimberConstants.kHasClimber = true;
                IntakeConstants.kHasIntake = true;
                ShooterConstants.kHasShooter = true;
                ShooterConstants.kHasTurret = true;
                LEDConstants.kHasLEDs = true;

                DriveConstants.kDebug = false;
                ControlPanelConstants.kDebug = false;
                ClimberConstants.kDebug = false;
                IntakeConstants.kDebug = true;
                ShooterConstants.kDebug = false;
                LEDConstants.kDebug = false;

                //CAN WIRING:
                ShooterConstants.kShooterMotor1Port = 18;
                ShooterConstants.kShooterMotor2Port = 6;
                // ShooterConstants.kHoodAngleServo1Port = 1; //PWM, not CAN
                // ShooterConstants.kHoodAngleServo2Port = 2; //PWM, not CAN
                ShooterConstants.kTurretMotorPort = 11;
                DriveConstants.kLeftMotor1Port = 12;
                DriveConstants.kLeftMotor2Port = 15;
                DriveConstants.kRightMotor1Port = 13;
                DriveConstants.kRightMotor2Port = 14;
                IntakeConstants.kIntakeFlipMotorPort = 10;
                IntakeConstants.kIntakeMotorPort = 4;
                IntakeConstants.kConveyorMotor1Port = 5;
                IntakeConstants.kConveyorMotor2Port = 8;
                ClimberConstants.kClimberMotor1Port = 16;
                ClimberConstants.kClimberMotor2Port = 17;
                ControlPanelConstants.kColorSensorPort = I2C.Port.kOnboard; //I2C, not CAN
                // ControlPanelConstants.kControlPanelRotatorMotorPort = 3;
                // ControlPanelConstants.kWheelRotatorMotorPort = -1;

                IntakeConstants.kIntakeColorSensor = 4; //Multiplexer I2C, not CAN
                IntakeConstants.kShooterColorSensor = 7; //Multiplexer I2C, not CAN
                // ControlPanelConstants.kControlPanelColorSensor = 3; //Multiplexer I2C, not CAN

                //DRIVE CONSTANTS
                DriveConstants.ksVolts = 0.195;
                DriveConstants.kvVoltSecondsPerMeter = 2.78;
                DriveConstants.kaVoltSecondsSquaredPerMeter = 0.381;
                DriveConstants.kPDriveVel = 2.57;
                DriveConstants.kTrackwidthMeters = 3.0;//TODO: put back;
                DriveConstants.kDriveKinematics = new DifferentialDriveKinematics(
                    DriveConstants.kTrackwidthMeters);
                DriveConstants.kFeedforward = new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter);
                DriveConstants.kLeftPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kRightPIDController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                DriveConstants.kMaxSpeedMetersPerSecond = 1.0;
                DriveConstants.kMaxAccelerationMetersPerSecondSquared = 1.0;
                DriveConstants.kMaxAngularSpeedRadiansPerSecond = 0.5 * Math.PI;//TODO: Put back/tune
                //DriveConstants.kRamseteB = 2;
                //DriveConstants.kRamseteZeta = 0.7;
                DriveConstants.kMaxRPM = 5700;
                DriveConstants.kWheelDiameter = 0.148;
                DriveConstants.kMotorGearsToWheelGears = 10.7;
                DriveConstants.kRevolutionsToMeters = Math.PI * DriveConstants.kWheelDiameter / DriveConstants.kMotorGearsToWheelGears;
                DriveConstants.kRPMtoMetersPerSecond = 
                    Math.PI * DriveConstants.kWheelDiameter / (60 * DriveConstants.kMotorGearsToWheelGears);
                DriveConstants.kGyroReversed = true;

                //INTAKE CONSTANTS
                IntakeConstants.kIntakeFlipP = 0.04;
                IntakeConstants.kIntakeFlipI = 1e-5;
                IntakeConstants.kIntakeFlipD = 0.2;
                IntakeConstants.kIntakeFlipIz = 0;
                IntakeConstants.kIntakeFlipFF = 0;
                IntakeConstants.kIntakeFlipMaxOutput = 1;
                IntakeConstants.kIntakeFlipMinOutput = -1;
                IntakeConstants.kIntakeFlipInEncoder = 0.0;
                IntakeConstants.kIntakeFlipOutEncoder = -55.0;
                
                IntakeConstants.kIntakeColorThesholdR = 930; //MAKE SURE THE GRAPH IS ORANGE AND NOT RED
                IntakeConstants.kIntakeColorThesholdG = 1600; //MAKE SURE THE GRAPH IS RED AND NOT GREEN
                IntakeConstants.kIntakeColorThesholdB = 0;
                IntakeConstants.kIntakeColorThesholdIR = 0;
                IntakeConstants.kIntakeColorThesholdProximity = 350;

                IntakeConstants.kShooterColorThesholdIR = 0;
                IntakeConstants.kShooterColorThesholdG = 0;
                IntakeConstants.kShooterColorThesholdB = 0;
                IntakeConstants.kShooterColorThesholdIR = 0;
                IntakeConstants.kShooterColorThesholdProximity = 200;

                //SHOOTER CONSTANTS
                
                ShooterConstants.kShooterP = 6e-2; 
                ShooterConstants.kShooterI = 0;
                ShooterConstants.kShooterD = 0; 
                ShooterConstants.kShooterIz = 0; 
                ShooterConstants.kShooterFF = 0.000015; 
                ShooterConstants.kShooterMaxOutput = 1; 
                ShooterConstants.kShooterMinOutput = -1;
                ShooterConstants.kMaxRPM = 5700;

                ShooterConstants.kMaxTurretAngle = 130;//135
                ShooterConstants.kMinTurretAngle = -130;//-135

                LEDConstants.kLEDPWMPort = 9;
            } break;
            case KOPChassis: {
                RobotName = "The KOP Chassis Robot";
                DriveConstants.kHasDriveTrain = true;
                DriveConstants.kHasGyro = true;
                DriveConstants.kHasPixy = false;

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
                //DriveConstants.kRamseteB = 2;
                //DriveConstants.kRamseteZeta = 0.7;
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
            case JeffBot: {
                RobotName = "The JEFF Robot";

                DriveConstants.kHasDriveTrain = true;
                DriveConstants.kHasGyro = false;
                DriveConstants.kHasPixy = true;
                ControlPanelConstants.kHasControlPanel = true;
                ShooterConstants.kHasShooter = true;

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
                DriveConstants.kLeftMotor1Port = 2;
                DriveConstants.kLeftMotor2Port = 1;
                DriveConstants.kRightMotor1Port = 4;
                DriveConstants.kRightMotor2Port = 3;
            } break;
            case TestBoard: {
                RobotName = "The Tester Boarder";
                LEDConstants.kHasLEDs = true;
                ShooterConstants.kHasShooter = false;
                DriveConstants.kHasDriveTrain = false;
                IntakeConstants.kHasIntake = false;
                DriveConstants.kHasGyro = false;
                DriveConstants.kHasPixy = false;
                ControlPanelConstants.kHasControlPanel = false;
                
                LEDConstants.kLEDPWMPort = 9;

            } break;
        }
        return robot;
    }
    public static class DriveConstants {
        public static boolean kHasDriveTrain = false;
        public static boolean kHasGyro = false;
        public static boolean kHasPixy = false;
        public static boolean kDebug = false;

        // Feedforward constants
        public static double ksVolts = 0.146;
        public static double kvVoltSecondsPerMeter = 2.17;
        public static double kaVoltSecondsSquaredPerMeter = 0.308;

        // Example value only - as above, this must be tuned for your drive!
        public static double kPDriveVel = 2.59;

        public static double kTrackwidthMeters = 1.5;//TODO: remove comment, calibrate0.66;
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
        public static enum Teleop {
            CheesyDrive,
            PixyDrive
        }
        public static Teleop teleop = Teleop.CheesyDrive;
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
        public static boolean kDebug = false;
        //What I2C port to plug the REV Color Sensor V3 into
        public static I2C.Port kColorSensorPort = I2C.Port.kOnboard;
        public static int kWheelRotatorMotorPort = 7;
        public static int kMechanismRotatorServoPort = -1; //PWM
        public static int kControlPanelColorSensor = 7; //I2C on multiplexer

        public static double kSliceWidthCounts = 32;
        public static double kRedColorDistanceThreshold = 0.17;
        public static double kBlueColorDistanceThreshold = 0.17;
        public static double kGreenColorDistanceThreshold = 0.07;
        public static double kYellowColorDistanceThreshold = 0.17;
        public static double kGotoPositionPower = 0.25;
        public static double kRotatePower = 0.25;

        //These values have been calculated by REV Robotics
        public static Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
        public static Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
        public static Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
        public static Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    }
    public static final class ShooterConstants {
        public static boolean kHasShooter = false;
        public static boolean kHasTurret = false;
        public static boolean kDebug = false;
        
        public static int kShooterMotor1Port = 7;//Shooter 1 = left
        public static int kShooterMotor2Port = 7;//Shooter 2 = right
        public static int kTurretMotorPort = 0;
        public static int kAngleScrewServoPort = 0;
        public static int kHoodAngleServoPort = -1;

        public static double kP = 0.02;
        public static double kFF = 0.075;
        public static double kChassisMultiplier = 2.0;
        public static double kAimingThreshold = 0.1;
        public static double kMaxTurretAngle = 60.0;
        public static double kMinTurretAngle = -60.0;

        public static double kShooterP = 6e-5; 
        public static double kShooterI = 0;
        public static double kShooterD = 0; 
        public static double kShooterIz = 0; 
        public static double kShooterFF = 0.000015; 
        public static double kShooterMaxOutput = 1; 
        public static double kShooterMinOutput = -1;
        public static double kMaxRPM = 5700;

        public static double kTurretP = 0.03;
        public static double kTurretI = 0;
        public static double kTurretD = 0;
        public static double kTurretIz = 0; 
        public static double kTurretFF = 0.000015; 
        public static double kTurretMaxOutput = 1; 
        public static double kTurretMinOutput = -1;

        public static double kHoodPositionReallyCloseWall = 1.9;
        public static double kHoodPositionAuto = 0;
        public static double kHoodPositionFar = -0.58;
        public static double kHoodPositionMid = -0.73;
        public static double kHoodPositionClose = -0.61;
    }
    public static final class IntakeConstants {
        public static boolean kHasIntake = false;
        public static boolean kHasIntakeColorSensor = false;
        public static boolean kDebug = false;
        
        public static int kIntakeFlipMotorPort = 7;
        public static int kIntakeMotorPort = 7;
        public static int kConveyorMotor1Port = 7;
        public static int kConveyorMotor2Port = 7;

        public static int kIntakeColorSensor = 7;
        public static int kShooterColorSensor = 7;

        public static double kIntakeFlipP = 0.0001;
        public static double kIntakeFlipI = 1e-4;
        public static double kIntakeFlipD = 1;
        public static double kIntakeFlipIz = 0;
        public static double kIntakeFlipFF = 0;
        public static double kIntakeFlipMaxOutput = 1;
        public static double kIntakeFlipMinOutput = -1;
        public static double kIntakeFlipInEncoder = 0.0;
        public static double kIntakeFlipOutEncoder = 0.0;
        
        public static double kIntakeColorThesholdR = 0;
        public static double kIntakeColorThesholdG = 0;
        public static double kIntakeColorThesholdB = 0;
        public static double kIntakeColorThesholdIR = 0;
        public static double kIntakeColorThesholdProximity = 0;
        public static double kShooterColorThesholdR = 0;
        public static double kShooterColorThesholdG = 0;
        public static double kShooterColorThesholdB = 0;
        public static double kShooterColorThesholdIR = 0;
        public static double kShooterColorThesholdProximity = 0;
    }
    public static final class ClimberConstants {
        public static boolean kHasClimber = false;
        public static boolean kDebug = false;
        
        public static int kClimberMotor1Port = 7;
        public static int kClimberMotor2Port = 7;
    }
    public static final class LEDConstants {
        public static boolean kHasLEDs = false;
        public static boolean kDebug = false;
        
        public static int kLEDCount = 72;//normally 72
        public static int kLEDPWMPort = -1;

        public static Color[] kRainbowSequence = {
            ColorShim.kRed,
            ColorShim.kOrange,
            ColorShim.kYellow,
            ColorShim.kGreen,
            ColorShim.kBlue,
            ColorShim.kPurple
        };
        public static Color[] kAmericaSequence = {
            ColorShim.kRed, 
            ColorShim.kWhite, 
            ColorShim.kBlue
        };
        public static Color[] kMainlyGreen = {
            ColorShim.kGreen, 
            ColorShim.kGreen, 
            ColorShim.kYellow,
        };
        public static Color[] kRandomSequence = {
            ColorShim.kRed,
            ColorShim.kWhite,
            ColorShim.kBlue,
            ColorShim.kGreen,
            ColorShim.kYellow,
            ColorShim.kOrange,
            ColorShim.kPurple
        };
    }
}

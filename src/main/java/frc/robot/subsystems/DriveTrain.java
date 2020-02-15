/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;
import java.util.function.BiConsumer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.ShuffleboardHelper;
import frc.robot.Tools;
import frc.robot.Constants.DriveConstants;
import frc.robot.Tools.MathTools;
import frc.robot.pixy.Pixy2;
import frc.robot.pixy.Pixy2CCC;
import frc.robot.pixy.Pixy2CCC.Block;
import frc.robot.pixy.links.SPILink;

public class DriveTrain extends SubsystemBase {
  // The motors on the left side of the drive.
  private final CANSparkMax m_leftFrontMotor;
  private final CANSparkMax m_leftBackMotor ;
  private final CANSparkMax m_rightFrontMotor;
  private final CANSparkMax m_rightBackMotor;
  private final Pixy2 pixy;
  
  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder;

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder;

  // The gyro sensor
  private final Gyro m_gyro;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveTrain() {
    if (DriveConstants.kHasDriveTrain) {
      m_leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
      m_leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
      m_rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
      m_rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
      m_rightEncoder = m_rightFrontMotor.getEncoder();
      m_leftEncoder = m_leftFrontMotor.getEncoder();
      m_drive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
  
      m_leftFrontMotor.restoreFactoryDefaults();
      m_leftBackMotor.restoreFactoryDefaults();
      m_rightFrontMotor.restoreFactoryDefaults();
      m_rightBackMotor.restoreFactoryDefaults();
      //Bind the front and back SparkMax's together using the follow() command
      m_leftBackMotor.follow(m_leftFrontMotor);
      m_rightBackMotor.follow(m_rightFrontMotor);
  
      // Sets the distance per pulse for the encoders
      m_leftEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
      m_rightEncoder.setPositionConversionFactor(DriveConstants.kRevolutionsToMeters);
      m_leftEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
      m_rightEncoder.setVelocityConversionFactor(DriveConstants.kRPMtoMetersPerSecond);
      resetEncoders();
      m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
      m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
      m_leftBackMotor.setIdleMode(IdleMode.kBrake);
      m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
      m_rightBackMotor.setIdleMode(IdleMode.kBrake);

      ShuffleboardHelper.addSparkMaxLayout("Drive Train Motors", Map.of(
        m_leftFrontMotor, "Left Front Motor",
        m_leftBackMotor, "Left Back Motor",
        m_rightFrontMotor, "Right Front Motor",
        m_rightBackMotor, "Right Back Motor"
      ));
    }
    else {
      m_leftFrontMotor = null;
      m_leftBackMotor = null;
      m_rightFrontMotor = null;
      m_rightBackMotor = null;
      m_odometry = null;
      m_drive = null;
      m_leftEncoder = null;
      m_rightEncoder = null;
    } 
    if (DriveConstants.kHasGyro) {
      m_gyro = new ADXRS450_Gyro();
      m_gyro.reset();
    }
    else {
      m_gyro = null;
    }
    if (DriveConstants.kHasPixy) {
      pixy = Pixy2.createInstance(new SPILink());
      pixy.init();
      //pixy.setLamp((byte) 0, (byte) 0); // Turns the LEDs on
      //pixy.setLED(200, 230, 30); // Sets the RGB LED to purple
    }
    else {
      pixy = null;
    }
    
  }

  public Block getBiggestBlock() {
    if (pixy == null)
      return null;
		// Gets the number of "blocks", identified targets, that match signature 1 on the Pixy2,
		// does not wait for new data if none is available,
		// and limits the number of returned blocks to 25, for a slight increase in efficiency
		int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		//System.out.println("Found " + blockCount + " blocks!"); // Reports number of blocks found
		if (blockCount <= 0) {
			return null; // If blocks were not found, stop processing
		}
		ArrayList<Block> blocks = pixy.getCCC().getBlocks(); // Gets a list of all blocks found by the Pixy2
		Block largestBlock = null;
		for (Block block : blocks) { // Loops through all blocks and finds the widest one
			if (largestBlock == null) {
				largestBlock = block;
			} else if (block.getWidth() > largestBlock.getWidth()) {
				largestBlock = block;
			}
		}
		return largestBlock;
  }
  
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if (DriveConstants.kHasDriveTrain && DriveConstants.kHasGyro) {
      m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),-m_rightEncoder.getPosition());
    }
    if (DriveConstants.kHasPixy) {
      Block b = getBiggestBlock();
      int pos = (b==null)?0:b.getX();
      SmartDashboard.putNumber("Pixy", pos);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void calculateArcadeDrive(double xSpeed, double rot) {
    var wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = DriveConstants.kFeedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = DriveConstants.kFeedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput = DriveConstants.kLeftPIDController.calculate(m_leftEncoder.getVelocity(),
        speeds.leftMetersPerSecond);
    double rightOutput = DriveConstants.kRightPIDController.calculate(-m_rightEncoder.getVelocity(),
        speeds.rightMetersPerSecond);
        System.out.println("L: "+speeds.leftMetersPerSecond+", R: "+speeds.rightMetersPerSecond+
        ", FFL: "+leftFeedforward+", FFR: "+rightFeedforward+
        ", LV: "+m_leftEncoder.getVelocity()+", RV: "+m_rightEncoder.getVelocity()+
        ", LO: "+leftOutput+", RO: "+rightOutput);
    
    tankDriveVolts(leftOutput + leftFeedforward, rightOutput + rightFeedforward);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    if (DriveConstants.kHasDriveTrain) {
      String debug = debug(leftVolts, rightVolts);
      //System.out.println(debug);
      m_leftFrontMotor.setVoltage(leftVolts);
      m_rightFrontMotor.setVoltage(-rightVolts);
      m_drive.feed();
    }
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + -m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    if (m_gyro == null)
      return 0;
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    if (m_gyro == null)
      return 0;
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public String debug(double l, double r){
    SmartDashboard.putNumber("Gyro", getHeading());
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", -m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", -m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Voltage", l);
    SmartDashboard.putNumber("Right Voltage", r);
    return String.format("Gyro(%f); Position(%f, %f); Velocity(%f, %f); Voltage(%f, %f);", 
      getHeading(), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition(),
      m_leftEncoder.getVelocity(), -m_rightEncoder.getVelocity(), l, r
    );
  }

  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;
  public static final double kDefaultDeadband = 0.02;

  private double m_quickStopThreshold = kDefaultQuickStopThreshold;
  private double m_quickStopAlpha = kDefaultQuickStopAlpha;
  private double m_quickStopAccumulator;
  protected double m_deadband = kDefaultDeadband;
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn, BiConsumer<Double, Double> output) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = MathTools.applyDeadband(xSpeed, m_deadband);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = MathTools.applyDeadband(zRotation, m_deadband);
    double angularPower;
    boolean overPower;
    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }
    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }
    output.accept(leftMotorOutput, rightMotorOutput);
  }
  
}
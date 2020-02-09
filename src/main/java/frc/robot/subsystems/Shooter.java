/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardHelper;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Tools.Toggle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

/**
 * The Shooter subsystem controls the shooter wheel, the rotating turret, the limelight, a hood angle and angle screw servos.
 * <p>Together, these three systems work to make the shooter operate as intended. 
 */
public class Shooter extends SubsystemBase {
  CANSparkMax shooterMotor1;
  CANSparkMax shooterMotor2;
  CANSparkMax turretMotor;
  CANEncoder turretEncoder;

  Servo hoodAngleServo;
  Servo angleScrewServo;

  NetworkTableEntry LEDMode;
  NetworkTableEntry cameraMode;

  NetworkTable table;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    if (ShooterConstants.kHasShooter) {
      //shooterMotor1 = new CANSparkMax(ShooterConstants.kShooterMotor1Port, MotorType.kBrushless);
      //shooterMotor2 = new CANSparkMax(ShooterConstants.kShooterMotor2Port, MotorType.kBrushless);
      //shooterMotor1.setIdleMode(IdleMode.kCoast);
      //shooterMotor2.setIdleMode(IdleMode.kCoast);
      //shooterMotor2.follow(shooterMotor1);
      
      turretMotor = new CANSparkMax(ShooterConstants.kTurretMotorPort, MotorType.kBrushless);
      turretMotor.setIdleMode(IdleMode.kBrake);
      turretEncoder = turretMotor.getEncoder();

      hoodAngleServo = new Servo(ShooterConstants.kHoodAngleServoPort);
      //angleScrewServo = new Servo(ShooterConstants.kAngleScrewServoPort);

      //Get the Network Tables for the limelight
      table = NetworkTableInstance.getDefault().getTable("limelight");

      //Get the Network Table Entry that controls the LEDs on the limelight so we can turn them on/off
      NetworkTableEntry ledMode = table.getEntry("ledMode");
      //Get the Network Table Entry that controls the camera stream output see we can change the PnP
      NetworkTableEntry stream = table.getEntry("stream");
      //Add togglers for the NT entries
      ShuffleboardHelper.AddToggle("Limelight", "LEDMode", ledMode::setDouble, new Toggle<Integer>(1, 3));
      ShuffleboardHelper.AddToggle("Limelight", "Stream PnP", stream::setDouble, new Toggle<Integer>(1, 2));

      //ShuffleboardHelper.AddOutput("Hood Angle", 0, 1, hoodAngleServo::set);
      ShuffleboardHelper.AddOutput("Hood Angle", 0, 1, (x)->{
        System.out.println("Servo output set to "+x);
        hoodAngleServo.set(x);
        System.out.println("Servo is actually at "+hoodAngleServo.get());
      });

      tv = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
    }
  }

  /**
   * Rotate the turret an amount from -1 to 1, only if we haven't past the thresholds
   * @param turn
   * @return True if at an edge and no power was set, or if there is no Shooter
   */
  public boolean rotate(double turn) {
    if (ShooterConstants.kHasShooter) {
      double currentAngle = encoderToDegrees(turretEncoder.getPosition());
      if (turn > 0) {
        //Want to rotate CLOCKWISE, which INCREASES encoder counts
        if (currentAngle < ShooterConstants.kMaxTurretAngle) {
          turretMotor.set(turn);
          return false;
        }
        else {
          turretMotor.set(0);
          return true;
        }
      }
      else {
        //Want to rotate COUNTER-CLOCKWISE, which DECREASES encoder counts
        if (currentAngle > ShooterConstants.kMinTurretAngle) {
          turretMotor.set(turn);
          return false;
        }
        else {
          turretMotor.set(0);
          return true;
        }
      }
    }
    else {
      return true;
    }
  }
  double encoderToDegrees(double counts) {
    return counts;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This method does everything to aim the turret using the limelight and determine when to shoot
   */

  /**
   * This method does everything to aim the turret using the limelight and shoot when ready.
   * If the turret cannot reach the desired angle, this method returns the speed the drice chassis needs to turn
   * @return 
   */
  public double AutoAimAndShoot() {
    if (ShooterConstants.kHasShooter) {
      double v = tv.getDouble(0.0);
      if (v > 0) {
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        SmartDashboard.putBoolean("LimelightTargetFound", true);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
  
        if (Math.abs(x) < ShooterConstants.kAimingThreshold) {
          //Target is within threshold, go ahead and shoot
  
          return 0;
        }
        else {
          //Target is not within threshold, rotate the turret
          double turn = ShooterConstants.kP*x + ((x>0)? -ShooterConstants.kFF : ShooterConstants.kFF);
          boolean driveChassis = rotate(turn);
          if (driveChassis) {
            return turn * ShooterConstants.kChassisMultiplier;
          }
          else {
            return 0;
          }
        }
      }
      else {
        SmartDashboard.putBoolean("LimelightTargetFound", false);
        return 0;
      }
    }
    else {
      return 0;
    }
  }
}

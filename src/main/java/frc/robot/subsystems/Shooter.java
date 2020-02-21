/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardHelper;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Tools.DataTools.Toggle;

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

  CANEncoder shooterEncoder1;
  CANEncoder shooterEncoder2;
  CANEncoder turretEncoder;

  CANPIDController shooterPID1;
  CANPIDController shooterPID2;

  Servo hoodAngleServo1;
  Servo hoodAngleServo2;

  NetworkTableEntry LEDMode;
  NetworkTableEntry cameraMode;

  NetworkTable table;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry pipeline;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    if (ShooterConstants.kHasShooter) {
      shooterMotor1 = new CANSparkMax(ShooterConstants.kShooterMotor1Port, MotorType.kBrushless);
      shooterMotor2 = new CANSparkMax(ShooterConstants.kShooterMotor2Port, MotorType.kBrushless);

      shooterEncoder1 = shooterMotor1.getEncoder();
      shooterEncoder2 = shooterMotor2.getEncoder();

      shooterMotor1.setIdleMode(IdleMode.kCoast);
      shooterMotor2.setIdleMode(IdleMode.kCoast);
      shooterMotor1.setInverted(true);
      shooterMotor2.setInverted(false);
      shooterPID1 = shooterMotor1.getPIDController();
      shooterPID2 = shooterMotor2.getPIDController();

      shooterPID1.setP(ShooterConstants.kShooterP);
      shooterPID1.setI(ShooterConstants.kShooterI);
      shooterPID1.setD(ShooterConstants.kShooterD);
      shooterPID1.setIZone(ShooterConstants.kShooterIz);
      shooterPID1.setFF(ShooterConstants.kShooterFF);
      shooterPID1.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);
      
      shooterPID2.setP(ShooterConstants.kShooterP);
      shooterPID2.setI(ShooterConstants.kShooterI);
      shooterPID2.setD(ShooterConstants.kShooterD);
      shooterPID2.setIZone(ShooterConstants.kShooterIz);
      shooterPID2.setFF(ShooterConstants.kShooterFF);
      shooterPID2.setOutputRange(ShooterConstants.kShooterMinOutput, ShooterConstants.kShooterMaxOutput);

      // turretMotor = new CANSparkMax(ShooterConstants.kTurretMotorPort, MotorType.kBrushless);
      // turretMotor.setIdleMode(IdleMode.kBrake);
      // turretEncoder = turretMotor.getEncoder();

      hoodAngleServo1 = new Servo(ShooterConstants.kHoodAngleServo1Port);
      hoodAngleServo2 = new Servo(ShooterConstants.kHoodAngleServo2Port);

      //Get the Network Tables for the limelight
      table = NetworkTableInstance.getDefault().getTable("limelight");

      //Get the Network Table Entry that controls the LEDs on the limelight so we can turn them on/off
      NetworkTableEntry ledMode = table.getEntry("ledMode");
      //Get the Network Table Entry that controls the camera stream output see we can change the PnP
      NetworkTableEntry stream = table.getEntry("stream");
      pipeline = table.getEntry("pipeline");
      setZoomPipeline(1);
      //Add togglers for the NT entries
      ShuffleboardHelper.AddToggle("Limelight", "LEDMode", ledMode::setDouble, new Toggle<Integer>(1, 3));
      //ShuffleboardHelper.AddToggle("Limelight", "Stream PnP", stream::setDouble, new Toggle<Integer>(1, 2));

      // ShuffleboardHelper.AddOutput("Hood Angle", 0, 1, hoodAngleServo::set);
      // ShuffleboardHelper.AddOutput(this, "Hood Angle", 0, 1, (x)->{
      //   hoodAngleServo1.set(x);
      //   //hoodAngleServo2.set(x);
      // });
      // ShuffleboardHelper.AddOutput(this, "Turret Power", 0, 1, (x)->{
      //   System.out.println("Set power for "+x);
      //   turretMotor.set(x);
      // });

      tv = table.getEntry("tv");
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");

      
      Shuffleboard.getTab("Shooter").addNumber("Shooter1 Vel", shooterEncoder1::getVelocity);
      Shuffleboard.getTab("Shooter").addNumber("Shooter2 Vel", shooterEncoder2::getVelocity);
      Shuffleboard.getTab("Shooter").addNumber("Limelight Distance", this::getLimelightDistance);
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
    // Calculate the limelight distance
  }
  public void setSpeed(double speed) {
    hoodAngleServo1.set(speed);
    hoodAngleServo2.set(1 - speed);
  }

  public double getLimelightDistance() {
    //System.out.println("v="+tv.getDouble(0.0));
    if (tv.getDouble(0.0) > 0.9) {
      double fixedCameraAngle = 27.9;//degrees was 17.6
      //27.9
      double cameraReadingAngle = ty.getDouble(0.0);//degrees
      double fixedCameraHeight = 0.923925;//meters
      double fixedGoalHeight = 2.49555;//meters
      double distance = (fixedGoalHeight - fixedCameraHeight) / 
      (Math.tan(Math.toRadians(cameraReadingAngle + fixedCameraAngle)));//meters
      return distance;
    }
    else {
      return -10.0;
    }
  }

  public void powerShooter(double speed) {
    speed = atSpeed()?0:speed;

    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }
  public void powerShooter(boolean power) {
    double speed = power? 4750 :0;
    if (power) {
      shooterPID1.setReference(speed, ControlType.kVelocity);
      shooterPID2.setReference(speed, ControlType.kVelocity);
    }
    else {
      shooterMotor1.set(0);
      shooterMotor2.set(0);
    }
  }
  public boolean atSpeed() {
    return shooterEncoder1.getVelocity() > 4750;
  }

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
        
        // SmartDashboard.putBoolean("LimelightTargetFound", true);
        // SmartDashboard.putNumber("LimelightX", x);
        // SmartDashboard.putNumber("LimelightY", y);
        // SmartDashboard.putNumber("LimelightArea", area);
  
        if (Math.abs(x) < ShooterConstants.kAimingThreshold) {
          //Target is within threshold, go ahead and shoot
  
          return 0;
        }
        else {
          //Target is not within threshold, rotate the turret
          double turn = ShooterConstants.kP*x + ((x>0)? -ShooterConstants.kFF : ShooterConstants.kFF);
          if (ShooterConstants.kHasTurret) {
            boolean driveChassis = rotate(turn);
            if (driveChassis) {
              return turn * ShooterConstants.kChassisMultiplier;
            }
            else {
              return 0;
            }
          }
          else {
            return turn;
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
  public void setZoomPipeline(double level) {
    pipeline.setDouble(level);
  }
}

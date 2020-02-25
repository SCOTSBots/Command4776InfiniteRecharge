/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ControlledServos;
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
  PIDController turretPID;

  ControlledServos hoodAngle;


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


      if (ShooterConstants.kHasTurret) {
        turretMotor = new CANSparkMax(ShooterConstants.kTurretMotorPort, MotorType.kBrushless);
        turretMotor.setInverted(true);
        turretMotor.setIdleMode(IdleMode.kBrake);
        turretEncoder = turretMotor.getEncoder();
        turretEncoder.setPosition(0);
        //turretPID = turretMotor.getPIDController();
        turretPID = new PIDController(ShooterConstants.kTurretP, ShooterConstants.kTurretI, ShooterConstants.kTurretD);
        turretPID.setSetpoint(0);
        // turretPID.setP(ShooterConstants.kTurretP);
        // turretPID.setI(ShooterConstants.kTurretI);
        // turretPID.setD(ShooterConstants.kTurretD);
        // turretPID.setIZone(ShooterConstants.kTurretIz);
        // turretPID.setFF(ShooterConstants.kTurretFF);
        // turretPID.setOutputRange(ShooterConstants.kTurretMinOutput, ShooterConstants.kTurretMaxOutput);

        if (ShooterConstants.kDebug) {
          Shuffleboard.getTab("Shooter").addNumber("Turret Encoder", turretEncoder::getPosition);  
        }
      } //End of turret

      hoodAngle = new ControlledServos(ShooterConstants.kHoodAngleServo1Port, ShooterConstants.kHoodAngleServo2Port);

      //Get the Network Tables for the limelight
      table = NetworkTableInstance.getDefault().getTable("limelight");

      //Get the Network Table Entry that controls the LEDs on the limelight so we can turn them on/off
      LEDMode = table.getEntry("ledMode");
      //Get the Network Table Entry that controls the camera stream output see we can change the PnP
      cameraMode = table.getEntry("stream");
      pipeline = table.getEntry("pipeline");
      setZoomPipeline(1);
      //Add togglers for the NT entries
      ShuffleboardHelper.AddToggle("Limelight", "LEDMode", LEDMode::setDouble, new Toggle<Integer>(1, 3));
      //ShuffleboardHelper.AddToggle("Limelight", "Stream PnP", cameraMode::setDouble, new Toggle<Integer>(1, 2));

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

      if (ShooterConstants.kDebug) {
        //Shuffleboard.getTab("Shooter").addNumber("Shooter1 Vel", shooterEncoder1::getVelocity);
        //Shuffleboard.getTab("Shooter").addNumber("Shooter2 Vel", shooterEncoder2::getVelocity);
        ShuffleboardHelper.addSparkMaxLayout("Shooters", Map.of(shooterMotor1,"Shooter1",shooterMotor2,"Shooter2"));
        Shuffleboard.getTab("Shooter").addNumber("Limelight Distance", this::getLimelightDistance);
        Shuffleboard.getTab("Shooter").addString("Side Mode", this::printSide);
        Shuffleboard.getTab("Shooter").addBoolean("Far Away", this::isFarAway);
        Shuffleboard.getTab("Shooter").addNumber("TX", ()->tx.getDouble(0));
        Shuffleboard.getTab("Shooter").addNumber("Servo 1", hoodAngle::getPosition);
      }
      Shuffleboard.getTab("Shooter").addNumber("avgshooter", this::getShooterSpeed);
      Shuffleboard.getTab("Shooter").addNumber("turret pos", turretEncoder::getPosition);
      enableLimelight(false);
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
      if (currentAngle+20 > ShooterConstants.kMaxTurretAngle)
        turn /= 2;
      if (currentAngle+20 < ShooterConstants.kMinTurretAngle)
        turn /= 2;
      if (turn > 0) {
        //Want to rotate CLOCKWISE, which INCREASES encoder counts
        if (currentAngle < ShooterConstants.kMaxTurretAngle) {
          turretMotor.set(turn);
          System.out.println("1");
          return false;
        }
        else {
          System.out.println("a");
          turretMotor.set(0);
          return true;
        }
      }
      else {
        //Want to rotate COUNTER-CLOCKWISE, which DECREASES encoder counts
        if (currentAngle > ShooterConstants.kMinTurretAngle) {
          turretMotor.set(turn);
          System.out.println("2");
          return false;
        }
        else {
          turretMotor.set(0);
          System.out.println("b");
          return true;
        }
      }
    }
    else {
      System.out.println("3");
      return true;
    }
  }
  public void powerTurret(double speed) {
    if (ShooterConstants.kHasTurret) {
      rotate(speed);
    }
  }
  double encoderToDegrees(double counts) {
    return counts;
  }
  int time=0;
  int delay=100;
  @Override
  public void periodic() {
    if (ShooterConstants.kHasShooter) {
      if (time++ > delay) {
        double mode = LEDMode.getDouble(0.0);
        if (mode < 0.1){
          enableLimelight(limelightOn);
        }
        else {
          if (limelightOn) {
            if (mode > 3.1 || mode < 2.9) {
              System.out.println("trying to turn it on");
              enableLimelight(limelightOn);
            }
          }
          else {
            if (mode > 1.1 || mode < 0.9) {
              System.out.println("trying to turn it off");
              enableLimelight(limelightOn);
            }
          }
        }
      }
    }
  }
  public void stopHood() {
    hoodAngle.stop();
  }
  public void hoodPosition(double target) {
    hoodAngle.gotoPosition(target);
  }

  public double getLimelightDistance() {
    //System.out.println("v="+tv.getDouble(0.0));
    if (tv.getDouble(0.0) > 0.9) {
      double fixedCameraAngle = 27.9;//degrees was 17.6
      //27.9
      double cameraReadingAngle = getZoomedLimelightY();//degrees
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
  public boolean isFarAway() {
    return getLimelightDistance() > 10;
  }
  double shooterSpeed = 5500;
  public void powerShooter(boolean power) {
    if (power) {
      shotABall();
      // shooterMotor1.set(1);
      // shooterMotor2.set(1);
      shooterSpeed = isFarAway()? 5500 : 5000;
      shooterPID1.setReference(shooterSpeed, ControlType.kVelocity);
      shooterPID2.setReference(shooterSpeed, ControlType.kVelocity);
    }
    else {
      shooterMotor1.set(0);
      shooterMotor2.set(0);
    }
  }
  /**
   * Determines whether the shooter is at speed to begin shooting
   * @return True if at speed, false if not.
   */
  public boolean atSpeed() {
    return shooterEncoder1.getVelocity() > (shooterSpeed - 250);
  }

  int direction;
  public void ZeroTurret() {
    if (ShooterConstants.kHasShooter && ShooterConstants.kHasTurret) {
      double turn = turretPID.calculate(turretEncoder.getPosition());
      System.out.println("turn: "+turn);
      // if (direction == 0) {
        
      // }
      // if (turretEncoder.getPosition() > 0 ^ right) {
        
      // }
      // else {

      // }
      rotate(turn);
    }
  }
  /**
   * This method does everything to aim the turret using the limelight and shoot when ready.
   * If the turret cannot reach the desired angle, this method returns the speed the drice chassis needs to turn
   * @return The speed the drive train would need to turn to align the robot if the turret is out of range.
   */
  public double AutoAimAndShoot() {
    if (ShooterConstants.kHasShooter) {
      if (!limelightOn)
        enableLimelight(true);
      double v = tv.getDouble(0.0);
      if (v > 0) {
        double x = tx.getDouble(0.0);

        if (Math.abs(x) < ShooterConstants.kAimingThreshold) {
          //Target is within threshold, go ahead and shoot
  
          return 0;
        }
        else {
          //Target is not within threshold, rotate the turret
          double turn = 0;
          //turn = ShooterConstants.kP*x + ((x>0)? -ShooterConstants.kFF : ShooterConstants.kFF);
          turn = turretPID.calculate(getToggledTarget()-x); //to the right use -2.5
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
  public void disableLimelight() {
    if (limelightOn)
      enableLimelight(false);
  }
  boolean limelightOn = false;
  public void enableLimelight(boolean on) {
    limelightOn = on;
    LEDMode.setDouble(on?3:1);
  }
  int zoomLevel = 1;
  /**
   * Set the zooming level of the limelight. Running this method switches pipeline, so expect a 1-2 second delay after running.
   * @param level The zoom level. 1 is normal, 2 is double, 3 is triple.
   */
  public void setZoomPipeline(int level) {
    zoomLevel = level;
    pipeline.setDouble(level);
  }
  /**
   * Get the y angle of the limelight based on which zoom level
   * @return The y angle, in degrees.
   */
  public double getZoomedLimelightY() {
    double y = ty.getDouble(0.0);
    switch (zoomLevel) {
      case 1:
        return y;
      case 2:
        return (y-13);
      case 3:
        return (y-19.5);
      default:
        return y;
    }
  }
  int side = 1;
  boolean wasPressed = false;
  /**
   * Toggle through the options to change, only toggling if you JUST pressed the button to prevent infinite toggling.
   * @param pressed Is the button currently pressed
   * @return The new side you are toggled at
   */
  public int toggleSide(boolean pressed) {
    if (pressed && !wasPressed) {
      side++;
      side %= 3;
    }
    wasPressed = pressed;
    return side;
  }
  /**
   * A handy dandy tool to print out what side you are toggled to
   * @return
   */
  public String printSide() {
    switch (side) {
      case 0:
        return "Left";
      case 1:
        return "Middle";
      case 2:
        return "Right";
      default:
        return "Unknown";
    }
  }
  /**
   * This method returns the offset the turret needs to be at if shooting at an angle
   * @return The offset. Set your reference point to this target minus the liemlight's x pos.
   */
  public double getToggledTarget() {
    //Behind the control panel = triple zoom with -3.5 at 5500rpm
    if (isFarAway()) {
      switch (side) {
        case 0:
          return 2.1;
        case 1:
          return 0;
        case 2:
          return -3.75;
        default:
          return 0;
      }
    }
    else {
      switch (side) {
        case 0:
          return 1.2;
        case 1:
          return 0;
        case 2:
          return -2.5;
        default:
          return 0;
      }
    }
  }
  double previousSpeed;
  boolean wasAtSpeed;
  /**
   * This method uses current speed and recent speeds of the shooter to determine if a ball just went through it.
   * @return True if a ball was just shot
   */
  public boolean shotABall() {
    double currentSpeed = getShooterSpeed();
    if (wasAtSpeed) {
      double delta = currentSpeed - previousSpeed;
      if (delta < -40) {
        //Shot one!
        System.out.println("Shot a ball!");
        previousSpeed = currentSpeed;
        wasAtSpeed = atSpeed();
        return true;
      }
      else if (delta < 0) {
        //System.out.println("not yet... its "+delta);
        previousSpeed = currentSpeed;
        return false;
      }
    }
    //System.out.println("fat chance");
    previousSpeed = currentSpeed;
    wasAtSpeed = atSpeed();
    return false;
  }
  /**
   * Gets the average speed of the shooter
   * @return The speed, in RPM.
   */
  public double getShooterSpeed() {
    return (shooterEncoder1.getVelocity() + shooterEncoder2.getVelocity()) / 2.0;
  }
}

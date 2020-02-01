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

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotType;

public class DriveTrain extends SubsystemBase {
  ADXRS450_Gyro gyro;
  //CANSparkMax left_shoot
  //CANSparkMax right_shoo
  //public SpeedController
  CANSparkMax left_front;
  CANSparkMax left_back;
  CANSparkMax right_front;
  CANSparkMax right_back;
  private DifferentialDrive driveTrain;

  //REV Examples:
  private CANPIDController left_pidController;
  private CANEncoder left_encoder;
  private CANPIDController right_pidController;
  private CANEncoder right_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
  
  //WPI Examples:
  public SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  public SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);
  public static final double radius = 0.234;
  public static final double maxRPM = 5700;
  public static final double wheelTurnsPerRevolution = 1 / 8.44;
  public static final double rpm2feetPerSecond = 2 * Math.PI * radius * wheelTurnsPerRevolution / 60;
  public static final double maxSpeed = maxRPM * rpm2feetPerSecond;
  public static final double trackWidth = 2.15;
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second



  private final DifferentialDriveKinematics m_kinematics
      = new DifferentialDriveKinematics(trackWidth);
  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  //public Servo servo
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(RobotType robot) {
    switch (robot){
      case Jeff: {
        gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        left_front = new CANSparkMax(1, MotorType.kBrushless);
        left_back = new CANSparkMax(2, MotorType.kBrushless);
        right_front = new CANSparkMax(3, MotorType.kBrushless);
        right_back = new CANSparkMax(4, MotorType.kBrushless);
        driveTrain = new DifferentialDrive(
          new SpeedControllerGroup(left_front, left_back), 
          new SpeedControllerGroup(right_front, right_back));
      } break;
      case WC: {
        gyro = null;//new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        left_front = new CANSparkMax(8, MotorType.kBrushless);
        left_back = new CANSparkMax(10, MotorType.kBrushless);
        right_front = new CANSparkMax(7, MotorType.kBrushless);
        right_back = new CANSparkMax(9, MotorType.kBrushless);
        left_back.follow(left_front);
        right_back.follow(right_front);
        // driveTrain = new DifferentialDrive(
        //   new SpeedControllerGroup(left_front, left_back), 
        //   new SpeedControllerGroup(right_front, right_back));
          driveTrain = new DifferentialDrive(left_front,right_front);
      } break;
      case KOP: {
        gyro = null;//new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        left_front = new CANSparkMax(4, MotorType.kBrushless);
        left_back = new CANSparkMax(5, MotorType.kBrushless);
        right_front = new CANSparkMax(3, MotorType.kBrushless);
        right_back = new CANSparkMax(6, MotorType.kBrushless);
        left_back.follow(left_front);
        right_back.follow(right_front);
        
        // driveTrain = new DifferentialDrive(
        //   new SpeedControllerGroup(left_front, left_back), 
        //   new SpeedControllerGroup(right_front, right_back));
          //driveTrain = new DifferentialDrive(left_front,right_front);
          right_front.setInverted(true);
          right_back.setInverted(true);
      } break;
      default: {

      }
    }
    left_front.setIdleMode(IdleMode.kBrake);
    left_back.setIdleMode(IdleMode.kBrake);
    right_front.setIdleMode(IdleMode.kBrake);
    right_back.setIdleMode(IdleMode.kBrake);
    double factor = 0.118 * 2 * 3.14 * radius;
    left_front.getEncoder().setPositionConversionFactor(factor);//feet = motor rotations * 0.118 ( wheel rotations / motor rotations) * (2*pi*radius / wheel rotation)
    left_back.getEncoder().setPositionConversionFactor(factor);
    right_front.getEncoder().setPositionConversionFactor(factor);
    right_back.getEncoder().setPositionConversionFactor(factor);
    speeds = new double[4];
    ss = new double[2];

    left_pidController = left_front.getPIDController();
    left_encoder = left_front.getEncoder();
    right_pidController = right_front.getPIDController();
    right_encoder = right_front.getEncoder();

    // PID coefficients
    kP = 0.236;
    kI = 0.847;
    kD = 0.165; 
    kIz = 0; 
    kFF = 0.236; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    // Smart Motion Coefficients
    
    maxVel = 20; // rpm
    maxAcc = 15;

    // set PID coefficients
    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(kIz);
    left_pidController.setFF(kFF);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);
    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(kIz);
    right_pidController.setFF(kFF);
    
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
   /* left_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    left_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    left_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    left_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
    right_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    right_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    right_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    right_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
*/
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 0);
    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes
    SmartDashboard.putBoolean("Mode", true);
  }
  public void cheesyDrive(double speed, double turn, boolean quickTurn){
    System.out.println("Power: "+speed+", turn: "+turn);
    driveTrain.curvatureDrive(speed, turn, quickTurn);
  }
  public void setSpeedsTankDrive(double left, double right) {
    System.out.println("Power: "+left+", "+right);
    driveTrain.tankDrive(left, right);
  }
  double[] speeds;
  public double[] getSpeeds_velocity(){
    speeds[0] = left_front.getEncoder().getVelocity();
    speeds[1] = left_back.getEncoder().getVelocity();
    speeds[2] = right_front.getEncoder().getVelocity();
    speeds[3] = right_back.getEncoder().getVelocity();
    return speeds;
  }
  public double[] getSpeeds_voltage(){
    speeds[0] = left_front.getBusVoltage();
    speeds[1] = left_back.getBusVoltage();
    speeds[2] = right_front.getBusVoltage();
    speeds[3] = right_back.getBusVoltage();
    return speeds;
  }
  public double[] getSpeeds_current(){
    speeds[0] = left_front.getOutputCurrent();
    speeds[1] = left_back.getOutputCurrent();
    speeds[2] = right_front.getOutputCurrent();
    speeds[3] = right_back.getOutputCurrent();
    return speeds;
  }
  public double[] getSpeeds_encoder(){
    speeds[0] = left_front.getEncoder().getPosition();
    speeds[1] = left_back.getEncoder().getPosition();
    speeds[2] = right_front.getEncoder().getPosition();
    speeds[3] = right_back.getEncoder().getPosition();
    return speeds;
  }
  public double getGyro(){
    return 0;//gyro.getAngle();
  }
  double[] ss;
  public double[] shooterSpeeds(){
    ss[0] = 0;//left_shooter.getEncoder().getVelocity();
    ss[1] = 0;//right_shooter.getEncoder().getVelocity();
    return ss;
  }
  public void set(double x) {}
  //@Override
  public void aperiodic() {
    // This method will be called once per scheduler run

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { left_pidController.setP(p); right_pidController.setP(p); kP = p; System.out.println("p"); }
    if((i != kI)) { left_pidController.setI(i);right_pidController.setI(i); kI = i; System.out.println("i"); }
    if((d != kD)) { left_pidController.setD(d);right_pidController.setD(d); kD = d; System.out.println("d"); }
    if((iz != kIz)) { left_pidController.setIZone(iz);right_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { left_pidController.setFF(ff);right_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      left_pidController.setOutputRange(min, max); right_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }/*
    if((maxV != maxVel)) { left_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; right_pidController.setSmartMotionMaxVelocity(maxV,0); }
    if((minV != minVel)) { left_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; right_pidController.setSmartMotionMinOutputVelocity(minV,0); }
    if((maxA != maxAcc)) { left_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; right_pidController.setSmartMotionMaxAccel(maxA,0); }
    if((allE != allowedErr)) { System.out.println("alle"); left_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; right_pidController.setSmartMotionAllowedClosedLoopError(allE,0); }
*/
    double setPoint, processVariable;
    boolean mode = SmartDashboard.getBoolean("Mode", false);
    if(mode) {
      setPoint = SmartDashboard.getNumber("Set Velocity", 0);
      setPoint = -RobotContainer.driverJoystick.getRawAxis(1)*maxRPM;
      double xSpeed = -RobotContainer.driverJoystick.getRawAxis(1);
      double zRotation = RobotContainer.driverJoystick.getRawAxis(4);
      boolean isQuickTurn = RobotContainer.driverJoystick.getRawButton(6);
      curvatureDrive(xSpeed, zRotation, isQuickTurn, (l,r)->{
        System.out.println("CD: "+l+", "+r);
        left_pidController.setReference(l*maxRPM, ControlType.kVelocity); 
        right_pidController.setReference(r*maxRPM, ControlType.kVelocity);
      });
      
      //left_pidController.setReference(setPoint*maxRPM, ControlType.kVelocity); 
      //right_pidController.setReference(setPoint*maxRPM, ControlType.kVelocity);
      processVariable = left_encoder.getVelocity();
    } else {
      setPoint = SmartDashboard.getNumber("Set Position", 0);
      setPoint = 5*RobotContainer.driverJoystick.getRawAxis(1);
      left_pidController.setReference(setPoint, ControlType.kSmartMotion); right_pidController.setReference(setPoint, ControlType.kSmartMotion);
      processVariable = left_encoder.getPosition();
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", left_front.getAppliedOutput());
  }
  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;
  public static final double kDefaultDeadband = 0.02;

  private double m_quickStopThreshold = kDefaultQuickStopThreshold;
  private double m_quickStopAlpha = kDefaultQuickStopAlpha;
  private double m_quickStopAccumulator;
  protected double m_deadband = kDefaultDeadband;
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn, TankDriver output) {
    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    xSpeed = applyDeadband(xSpeed, m_deadband);
    zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
    zRotation = applyDeadband(zRotation, m_deadband);
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
    output.set(leftMotorOutput, rightMotorOutput);
  }

  public static double applyDeadband(double value, double deadband){
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
  public interface TankDriver{
    public void set(double left, double right);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    final double leftOutput = m_leftPIDController.calculate(left_encoder.getVelocity(),
        speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(right_encoder.getVelocity(),
        speeds.rightMetersPerSecond);
    left_front.setVoltage(leftOutput + leftFeedforward);
    right_front.setVoltage(rightOutput + rightFeedforward);
  }
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }
}

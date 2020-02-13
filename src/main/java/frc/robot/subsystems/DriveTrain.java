/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotType;

public class DriveTrain extends SubsystemBase {
  ADXRS450_Gyro gyro;
  //CANSparkMax left_shoot
  //CANSparkMax right_shoo
  //public SpeedController
  CANSparkMax mleft_front;
  CANSparkMax mleft_back;
  CANSparkMax mright_front;
  CANSparkMax mright_back;  
  CANSparkMax melevator1;
  CANSparkMax melevator2;
  CANSparkMax mshooter1;
  CANSparkMax mshooter2;
  CANSparkMax mturretRotate;
  //hood used two servos
  private DifferentialDrive driveTrain;
  //public Servo servo
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain(RobotType robot) {
    switch (robot){
      case Jeff: {
        gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        mleft_front = new CANSparkMax(2, MotorType.kBrushless);
        mleft_back = new CANSparkMax(1, MotorType.kBrushless);
        mright_front = new CANSparkMax(4, MotorType.kBrushless);
        mright_back = new CANSparkMax(3, MotorType.kBrushless);
        driveTrain = new DifferentialDrive(
          new SpeedControllerGroup(mleft_front, mleft_back), 
          new SpeedControllerGroup(mright_front, mright_back));
      } break;
      case WC: {
        gyro = null;//new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        mleft_front = new CANSparkMax(8, MotorType.kBrushless);
        mleft_back = new CANSparkMax(10, MotorType.kBrushless);
        mright_front = new CANSparkMax(7, MotorType.kBrushless);
        mright_back = new CANSparkMax(9, MotorType.kBrushless);
        driveTrain = new DifferentialDrive(
          new SpeedControllerGroup(mleft_front, mleft_back), 
          new SpeedControllerGroup(mright_front, mright_back));
      } break;
      case KOP: {
        gyro = null;//new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
        mleft_front = new CANSparkMax(4, MotorType.kBrushless);
        mleft_back = new CANSparkMax(5, MotorType.kBrushless);
        mright_front = new CANSparkMax(3, MotorType.kBrushless);
        mright_back = new CANSparkMax(6, MotorType.kBrushless);
        driveTrain = new DifferentialDrive(
          new SpeedControllerGroup(mleft_front, mleft_back), 
          new SpeedControllerGroup(mright_front, mright_back));
      } break;
      case PracticeBot: {
        mleft_front = new CANSparkMax(0, MotorType.kBrushless);
        mleft_front = new CANSparkMax(0, MotorType.kBrushless);
        mright_front = new CANSparkMax(0, MotorType.kBrushless);
        mright_back = new CANSparkMax(0, MotorType.kBrushless);
        melevator1 = new CANSparkMax(0, MotorType.kBrushless);
        melevator2 = new CANSparkMax(0, MotorType.kBrushless);
        mshooter1 = new CANSparkMax(0, MotorType.kBrushless);
        mshooter2 = new CANSparkMax(0, MotorType.kBrushless);
        mturretRotate = new CANSparkMax(0, MotorType.kBrushless);
        driveTrain = new DifferentialDrive(
          new SpeedControllerGroup(mleft_front, mleft_back),
          new SpeedControllerGroup(mright_front, mright_back));

      } break;
      default: {

      }
    }
    speeds = new double[4];
    ss = new double[2];
  }
  public void cheesyDrive(double speed, double turn, boolean quickTurn){
    System.out.println("Power: "+speed);
    driveTrain.curvatureDrive(speed, turn, quickTurn);  
  }
  public void setSpeedsTankDrive(double d, double e) {
    //System.out.println("Power: "+left+", "+right);
    driveTrain.tankDrive(d, e);
  }
  double[] speeds;
  public double[] getSpeeds_velocity(){
    speeds[0] = mleft_front.getEncoder().getVelocity();
    speeds[1] = mleft_back.getEncoder().getVelocity();
    speeds[2] = mright_front.getEncoder().getVelocity();
    speeds[3] = mright_back.getEncoder().getVelocity();
    return speeds;
  }
  public double[] getSpeeds_voltage(){
    speeds[0] = mleft_front.getBusVoltage();
    speeds[1] = mleft_back.getBusVoltage();
    speeds[2] = mright_front.getBusVoltage();
    speeds[3] = mright_back.getBusVoltage();
    return speeds;
  }
  public double[] getSpeeds_current(){
    speeds[0] = mleft_front.getOutputCurrent();
    speeds[1] = mleft_back.getOutputCurrent();
    speeds[2] = mright_front.getOutputCurrent();
    speeds[3] = mright_back.getOutputCurrent();
    return speeds;
  }
  public double getGyro(){
    return gyro.getAngle();
  }
  double[] ss;
  public double[] shooterSpeeds(){
    ss[0] = 0;//left_shooter.getEncoder().getVelocity();
    ss[1] = 0;//right_shooter.getEncoder().getVelocity();
    return ss;
  }
  public void set(double power){
    //servo.set(power);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

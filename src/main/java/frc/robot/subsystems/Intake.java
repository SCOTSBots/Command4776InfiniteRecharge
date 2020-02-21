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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MultiplexedColorSensor;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  CANSparkMax conveyorMotor1;
  CANSparkMax conveyorMotor2;
  CANSparkMax intakeFlipMotor;

  CANEncoder intakeFlipEncoder;
  CANEncoder conveyorMotor1Encoder;
  CANEncoder conveyorMotor2Encoder;

  CANPIDController intakeFlipPID;

  MultiplexedColorSensor intakeColorSensor;
  MultiplexedColorSensor shooterColorSensor;

  boolean flipperOut;
  int ballsInRobot;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    if (IntakeConstants.kHasIntake) {
      ballsInRobot = 0;

      intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
      intakeFlipMotor = new CANSparkMax(IntakeConstants.kIntakeFlipMotorPort, MotorType.kBrushless);
      intakeFlipEncoder = intakeFlipMotor.getEncoder();
      resetFlipper(0);
      intakeFlipPID = intakeFlipMotor.getPIDController();
      intakeFlipPID.setP(IntakeConstants.kIntakeFlipP);
      intakeFlipPID.setI(IntakeConstants.kIntakeFlipI);
      intakeFlipPID.setD(IntakeConstants.kIntakeFlipD);
      intakeFlipPID.setIZone(IntakeConstants.kIntakeFlipIz);
      intakeFlipPID.setFF(IntakeConstants.kIntakeFlipFF);
      intakeFlipPID.setOutputRange(IntakeConstants.kIntakeFlipMinOutput, IntakeConstants.kIntakeFlipMaxOutput);
      
      conveyorMotor1 = new CANSparkMax(IntakeConstants.kConveyorMotor1Port, MotorType.kBrushless);
      conveyorMotor2 = new CANSparkMax(IntakeConstants.kConveyorMotor2Port, MotorType.kBrushless);
      conveyorMotor1.setInverted(false);
      conveyorMotor2.setInverted(true);

      conveyorMotor1Encoder = conveyorMotor1.getEncoder();
      conveyorMotor2Encoder = conveyorMotor2.getEncoder();

      flipperOut = false;
      Shuffleboard.getTab("Intake").addNumber("C1 Pos", conveyorMotor1Encoder::getPosition);
      Shuffleboard.getTab("Intake").addNumber("C2 Pos", conveyorMotor2Encoder::getPosition);
      Shuffleboard.getTab("Intake").addNumber("Flipper Pos", intakeFlipEncoder::getPosition);
      // color6 = new MultiplexedColorSensor(I2C.Port.kOnboard, 6);
      // color5 = new MultiplexedColorSensor(I2C.Port.kOnboard, 5);
      
      intakeColorSensor = new MultiplexedColorSensor(I2C.Port.kOnboard, 4);
      Shuffleboard.getTab("Intake").addNumber("Color4 Red", intakeColorSensor::getRed);
      Shuffleboard.getTab("Intake").addNumber("Color4 Green", intakeColorSensor::getGreen);
      Shuffleboard.getTab("Intake").addNumber("Color4 Blue", intakeColorSensor::getBlue);
      Shuffleboard.getTab("Intake").addNumber("Color4 Proximity", intakeColorSensor::getProximity);
      Shuffleboard.getTab("Intake").addNumber("Color4 IR", intakeColorSensor::getIR);
      Shuffleboard.getTab("Intake").addBoolean("Color4 INTAKE", this::ballInIntake);
    }
  }
  /**
   * Set the flipper to position
   * @param down - True if going to intake balls, False if pulling it up for defense.
   */
  public void setFlipper(boolean down) {
    flipperOut = down;
    double val = down?IntakeConstants.kIntakeFlipOutEncoder:IntakeConstants.kIntakeFlipInEncoder;
    intakeFlipPID.setReference(val, ControlType.kPosition);
  }
  public boolean toggleFlipper(){
    flipperOut = !flipperOut;
    setFlipper(flipperOut);
    return flipperOut;
  }
  public void resetFlipper(double newPosition){
    intakeFlipEncoder.setPosition(newPosition);
  }
  public void powerFlipper(double speed){
    System.out.println("PFR: "+speed);
    intakeFlipMotor.set(speed);
  }

  public double getConveyorPosition() {
    return conveyorMotor1Encoder.getPosition();
  }

  public void reading() {
    if (IntakeConstants.kHasIntake) {
      int d6 = -1;//color6.getProximity();
      int d5 = -1;//color5.getProximity();
      int d4 = intakeColorSensor.getProximity();
      int r = intakeColorSensor.getRed();
      int g = intakeColorSensor.getGreen();
      int b = intakeColorSensor.getBlue();
      System.out.println(String.format("Color: (%d, %d, %d), Dist=%d", r,g,b,d4));
    }
  }

  public void powerIntake(double speed) {
    if (IntakeConstants.kHasIntake) {
      intakeMotor.set(speed);
    }
  }
  public void powerConveyor(double speed) {
    if (IntakeConstants.kHasIntake) {
      conveyorMotor1.set(speed);
      conveyorMotor2.set(speed);
    }
  }
  public int getBallsInRobot() {
    return ballsInRobot;
  }
  public void addBallsInRobot(int add) {
    ballsInRobot += add;
  }
  public void setBallsInRobot(int newBalls) {
    ballsInRobot = newBalls;
  }
  public boolean ballInIntake() {
    boolean r = intakeColorSensor.getRed() > IntakeConstants.kIntakeColorThesholdR;
    boolean g = intakeColorSensor.getGreen() > IntakeConstants.kIntakeColorThesholdG;
    boolean b = true;//intakeColorSensor.getBlue() > IntakeConstants.kIntakeColorThesholdB;
    boolean ir = true;//intakeColorSensor.getIR() > IntakeConstants.kIntakeColorThesholdIR;
    boolean proximity = true;//intakeColorSensor.getProximity() > IntakeConstants.kIntakeColorThesholdProximity;
    boolean in = r && g && b && ir && proximity;

    return in;
  }
  public boolean ballInShooter() {
    boolean r = shooterColorSensor.getRed() > IntakeConstants.kShooterColorThesholdR;
    boolean g = shooterColorSensor.getGreen() > IntakeConstants.kShooterColorThesholdG;
    boolean b = shooterColorSensor.getBlue() > IntakeConstants.kShooterColorThesholdB;
    boolean ir = shooterColorSensor.getIR() > IntakeConstants.kShooterColorThesholdIR;
    boolean proximity = shooterColorSensor.getProximity() > IntakeConstants.kShooterColorThesholdProximity;
    return r && g && b && ir && proximity;
  }

  @Override
  public void periodic() {
    //reading();
  }
}

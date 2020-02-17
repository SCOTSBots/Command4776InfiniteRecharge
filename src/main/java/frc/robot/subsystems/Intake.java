/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MultiplexedColorSensor;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  CANSparkMax conveyorMotor1;
  CANSparkMax conveyorMotor2;
  CANSparkMax intakeFlipMotor;
  I2C mux;
  Ultrasonic sonic;
  //MultiplexerColorSensorV3 color;
  /*ColorSensorV3 color6;
  ColorSensorV3 color5;
  ColorSensorV3 color4;*/
  MultiplexedColorSensor color6;
  MultiplexedColorSensor color5;
  MultiplexedColorSensor color4;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    if (IntakeConstants.kHasIntake) {
      // intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
      // intakeFlipMotor = new CANSparkMax(IntakeConstants.kIntakeFlipMotorPort, MotorType.kBrushed);
      // conveyorMotor1 = new CANSparkMax(IntakeConstants.kConveyorMotor1Port, MotorType.kBrushless);
      // conveyorMotor2 = new CANSparkMax(IntakeConstants.kConveyorMotor2Port, MotorType.kBrushless);
      /*mux = new I2C(I2C.Port.kOnboard, 0x70);
      setPort(6);
      //color = new MultiplexerColorSensorV3(7);
      color6 = new ColorSensorV3(I2C.Port.kOnboard);
      setPort(5);
      color5 = new ColorSensorV3(I2C.Port.kOnboard);
      setPort(4);
      color4 = new ColorSensorV3(I2C.Port.kOnboard);*/
      color6 = new MultiplexedColorSensor(I2C.Port.kOnboard, 6);
      color5 = new MultiplexedColorSensor(I2C.Port.kOnboard, 5);
      color4 = new MultiplexedColorSensor(I2C.Port.kOnboard, 4);
      //color = new ColorSensorV3(I2C.Port.kOnboard);
      /*for (int t=0; t<8; t++) {
        setPort(t);
        System.out.println("TCA Port #"+t);
   
        for (int addr = 0; addr<=127; addr++) {
          if (addr == 0x70) continue;
        
          int data = 42;
          
          if (! mux.write(addr, data)) {
             System.out.println("Found I2C 0x"+Integer.toHexString(addr));
          }
        }
      }*/
    }
    sonic = new Ultrasonic(5, 4, Unit.kInches);
    sonic.setAutomaticMode(true);
    //Shuffleboard.getTab("Test Tabe").addNumber("US Distance", sonic::getRangeInches);
  
    //Shuffleboard.getTab("Example tab").add(sonic);
  }
  
  void setPort(int i) {
    //if (i > 7) return;
    //mux.write(0x70, 1 << i);
    //mux.close();
  }
  public void reading() {
    setPort(6);
    int d6 = color6.getProximity();
    setPort(5);
    int d5 = color5.getProximity();
    setPort(4);
    int d4 = color4.getProximity();
    System.out.println("Prox6: "+d6+", Prox5: "+d5+", Prox4: "+d4);
    /*
    byte[] data = new byte[32];
    mux.read(0x52, data.length, data);
    System.out.println(data.toString());*/
  }

  public void powerIntake(double power) {
    if (IntakeConstants.kHasIntake) {
      intakeMotor.set(power);
      conveyorMotor1.set(power);
      conveyorMotor2.set(power);
    }
  }

  @Override
  public void periodic() {
    if (IntakeConstants.kHasIntake) {
      reading();
    }
    // This method will be called once per scheduler run
    // System.out.println("US: "+sonic.getRangeInches());
  }
}

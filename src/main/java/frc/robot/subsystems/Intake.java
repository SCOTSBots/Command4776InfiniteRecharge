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
import frc.robot.MultiplexerColorSensorV3;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  CANSparkMax conveyorMotor1;
  CANSparkMax conveyorMotor2;
  CANSparkMax intakeFlipMotor;
  I2C mux;
  Ultrasonic sonic;
  MultiplexerColorSensorV3 color;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    if (IntakeConstants.kHasIntake) {
      // intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
      // intakeFlipMotor = new CANSparkMax(IntakeConstants.kIntakeFlipMotorPort, MotorType.kBrushed);
      // conveyorMotor1 = new CANSparkMax(IntakeConstants.kConveyorMotor1Port, MotorType.kBrushless);
      // conveyorMotor2 = new CANSparkMax(IntakeConstants.kConveyorMotor2Port, MotorType.kBrushless);
      //mux = new I2C(I2C.Port.kOnboard, 0x70);
      color = new MultiplexerColorSensorV3(7);
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
    if (i > 7) return;
    mux.write(0x70, 1 << i);
    mux.close();
  }
  public void reading() {
    //setPort(7);
    System.out.println("Prox: "+color.getProximity());
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
    //reading();
    // This method will be called once per scheduler run
    // System.out.println("US: "+sonic.getRangeInches());
  }
}

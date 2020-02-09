/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor;
  CANSparkMax conveyorMotor1;
  CANSparkMax conveyorMotor2;

  /**
   * Creates a new Intake.
   */
  public Intake() {
    if (IntakeConstants.kHasIntake) {
      intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
      conveyorMotor1 = new CANSparkMax(IntakeConstants.kConveyorMotor1Port, MotorType.kBrushless);
      conveyorMotor2 = new CANSparkMax(IntakeConstants.kConveyorMotor2Port, MotorType.kBrushless);
    }
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
    // This method will be called once per scheduler run
    
  }
}

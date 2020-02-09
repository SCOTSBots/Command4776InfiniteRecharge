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
import frc.robot.Constants.ClimberConstants;

/**
 * This class operates the climbing motors.
 */
public class Climber extends SubsystemBase {
  CANSparkMax climbMotor1;
  CANSparkMax climbMotor2;

  /**
   * Creates a new Climber.
   */
  public Climber() {
    if (ClimberConstants.kHasClimber) {
      climbMotor1 = new CANSparkMax(ClimberConstants.kClimberMotor1Port, MotorType.kBrushless);
      climbMotor2 = new CANSparkMax(ClimberConstants.kClimberMotor2Port, MotorType.kBrushless);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

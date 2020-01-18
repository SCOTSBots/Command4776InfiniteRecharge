/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  
  private DifferentialDrive driveTrain = new DifferentialDrive(
    new SpeedControllerGroup(new CANSparkMax(1, MotorType.kBrushless), new CANSparkMax(2, MotorType.kBrushless)), 
    new SpeedControllerGroup(new CANSparkMax(3, MotorType.kBrushless), new CANSparkMax(4, MotorType.kBrushless)));
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {

  }
  public void cheesyDrive(double speed, double turn, boolean quickTurn){
    driveTrain.curvatureDrive(speed, turn, quickTurn);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

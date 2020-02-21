/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class ControlledServo {
  Servo servo;
  Timer timer;
  double position;
  double prevTime;
  double speed;
  public ControlledServo(int port) {
    servo = new Servo(port);
    timer = new Timer();
    timer.reset();
    position = 0;
    speed = 0;
  }
  public ControlledServo(int port, double position) {
    this(port);
    setPosition(position);
  }
  public void setPosition(double position) {
    this.position = position;
  }
  public void gotoPosition(double target) {
    double time = timer.get();
    if (target > position) {
      //TODO: Finish controlled servo
    }
    prevTime = time;
  }
  public void set(double speed) {
    this.speed = speed;
    
  }
}

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
 * This is a continuos servo using a timer to mock an encoder to allow position control.
 */
public class ControlledServos {
  Servo servo1;
  Servo servo2;
  Timer timer;
  double position;
  double prevTime;
  double speed;
  double threshold = 0.05;
  /**
   * Main constructor. Just specify the port and BOOM, all done.
   * @param port - The PWM port the servo is plugged into.
   */
  public ControlledServos(int port1, int port2) {
    servo1 = new Servo(port1);
    servo2 = new Servo(port2);
    timer = new Timer();
    timer.reset();
    timer.start();
    position = 0;
    speed = 0;
  }
  public ControlledServos(int port1, int port2, double position) {
    this(port1, port2);
    setPosition(position);
  }
  public void setPosition(double position) {
    this.position = position;
  }
  public void gotoPosition(double target) {
    double time = timer.get(); //Get the current time
    position += speed * (time - prevTime); //Update "encoders"
    if (Math.abs(target - position) < threshold) {
      stop();
    }
    else {
      set(target > position? 1:-1);
    }
    prevTime = time;
  }
  public void stop() {
    servo1.stopMotor();
    servo2.stopMotor();
  }
  /**
   * Set the servo speed (like a normal motor).
   * @param speed - Goes from 1 to -1. Not 0 to 0.5 to 1!!!!!!
   */
  public void set(double speed) {
    double time = timer.get(); //Get the current time
    position += speed * (time - prevTime); //Update "encoders"
    prevTime = time;
    this.speed = speed;
    // System.out.println(speed/2 + 0.5);
    
    servo1.set(speed/2 + 0.5);
    servo2.set(0.5 - speed/2); //Invert servo2
  }
  public double getPosition() {
    return position;
  }
}

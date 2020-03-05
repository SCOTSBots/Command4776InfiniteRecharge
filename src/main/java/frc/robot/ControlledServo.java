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
public class ControlledServo {
  Servo servo;
  Timer timer;
  double position;
  double prevTime;
  double speed;
  double threshold = 0.05;
  /**
   * Main constructor. Just specify the port and BOOM, all done.
   * @param port - The PWM port the servo is plugged into.
   */
  public ControlledServo(int port) {
    servo = new Servo(port);
    timer = new Timer();
    timer.reset();
    timer.start();
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
    double time = timer.get(); //Get the current time
    position += speed * (time - prevTime); //Update "encoders"
    prevTime = time;
    speed = 0;
    servo.stopMotor();
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
    
    servo.set(speed/2 + 0.5); //Convert the scale of (-1, 1) to (0, 1)
  }
  public double getPosition() {
    return position;
  }
}

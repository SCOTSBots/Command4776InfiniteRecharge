/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Use the classes in this class for easy use.
 */
public class Tools {
  public static double deadzone(double x) {
        if (Math.abs(x) < 0.05) {
            return 0;
        } else {
            return x;
        }
    }

  public static double applyDeadband(double value, double deadband){
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

    public static class Pair<T1, T2> {
        private T1 a;
        private T2 b;
        public Pair(T1 a, T2 b) {
            this.a = a;
            this.b = b;
        }
        public T1 getT1() {
            return a;
        }
        public T2 getT2() {
            return b;
        }
        public T1 setT1(T1 a) {
            this.a = a;
            return a;
        }
        public T2 setT2(T2 b) {
            this.b = b;
            return b;
        }
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.ControlPanelConstants;

/**
 * Use the classes in this class for easy use.
 */
public class Tools {

  public static class DataTools {
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
    public static class Cycler<T> {
      private T[] items;
      int current;
      public T get() {
        return items[current];
      }
      public T cycle() {
        T t = get();
        current++;
        if (current >= items.length) {
          current = 0;
        }
        return t;
      }
      public Cycler(T[] newItems) {
        this.items = newItems;
        current = 0;
      }
    }
    public static class Toggle<T> {
      private T a;
      private T b;
      boolean current;
      /**
       * When you swap, you are getting the old value, so the next value (even with get) will be different.
       * @return The value previously held
       */
      public T swap() {
          T t = get();
          current = !current;
          return t;
      }
      public T get() {
          if (current)
              return b;
          return a;
      }
      /**
       * Create a toggle. The first get() returns a.
       * @param a - the first value
       * @param b - the second value
       */
      public Toggle(T a, T b) {
          this.a = a;
          this.b = a;
          current = false;
      }
    }
    public static class Toggleable<T> {
      Toggle<T> input;
      Consumer<T> output;
      public void toggle() {
        output.accept(input.swap());
      }
      public void init() {

      }
      public Toggleable(Toggle<T> t, Consumer<T> c) {
        input = t;
        output = c;
      }
    }
  }
  
  public static class MathTools {
    public static double deadzone(double x) {
      if (Math.abs(x) < 0.05) {
        return 0;
      } else {
        return x;
      }
    }
    public static double applyDeadband(double value, double deadband) {
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
    public static double absoluteDecimal(double position, double total) {
      return (position % total) / total;
    }
  }
  
  public static class ColorTools {

    public static enum BaseColor {
      Red,
      Green,
      Blue,
      Yellow,
      Unknown
    }
    public static String colorToString(Color color) {
      String colorString = "Error";
      if (color == ControlPanelConstants.kBlueTarget) {
        colorString = "Blue";
      } else if (color == ControlPanelConstants.kRedTarget) {
        colorString = "Red";
      } else if (color == ControlPanelConstants.kGreenTarget) {
        colorString = "Green";
      } else if (color == ControlPanelConstants.kYellowTarget) {
        colorString = "Yellow";
      } else {
        colorString = "Unknown";
      }
      return colorString;
    }
    public static double colorDistance(Color color1, Color color2) {
      if (color1 == null || color2 == null) return -1;
      
      double red = color1.red - color2.red;
      double green = color1.green - color2.green;
      double blue = color1.blue - color2.blue;
      
      return Math.sqrt(red*red + green*green + blue*blue);
    }
    /**
     * Simulates a rotation. Rotate one color slice, and return what the new color would be
     * @param currentColor - Where you are before rotating
     * @param counterClockWise - True if rotating counter clockwise
     * @param reverse - True if you want what happened BEFORE (use this to guess the previous color)
     * @return
     */
    public static BaseColor rotateColor(BaseColor currentColor, boolean counterClockWise, boolean reverse) {
      if (counterClockWise ^ reverse) {
        switch (currentColor) {
          case Red: return BaseColor.Yellow;
          case Green: return BaseColor.Red;
          case Blue: return BaseColor.Green;
          case Yellow: return BaseColor.Blue;
          case Unknown: return BaseColor.Unknown;
        }
      }
      else {
        switch (currentColor) {
          case Red: return BaseColor.Green;
          case Green: return BaseColor.Blue;
          case Blue: return BaseColor.Yellow;
          case Yellow: return BaseColor.Red;
          case Unknown: return BaseColor.Unknown;
        }
      }
      return BaseColor.Unknown;
    }

    public static BaseColor robotColorToSensorColor(BaseColor currentColor) {
      switch (currentColor) {
        case Red: return BaseColor.Blue;
        case Green: return BaseColor.Yellow;
        case Blue: return BaseColor.Red;
        case Yellow: return BaseColor.Green;
        case Unknown: return BaseColor.Unknown;
      }
      return BaseColor.Unknown;
    }
  
  }

}
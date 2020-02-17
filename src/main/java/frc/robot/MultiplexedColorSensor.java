/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * This is a wrapper for the REV Color Sensor V3, allowing multiple ones to be
 * used on a multiplexer board (this was tested using the Adafruit TCA9548A).
 * 
 * <p>
 * If you want to use this class, make sure to import REV's color sensor API.
 * </p>
 * 
 * <p>
 * This helper class was developed by Team 4776. <b>Go S.C.O.T.S. Bots!</b>
 * </p>
 */
public class MultiplexedColorSensor {
  // Change this if your multiplexer has a different address. This is the TCA9548A's default address.
  private final int kMultiplexerAddress = 0x70;

  // The multiplexer I2C is static because it needs to be used for ALL of the multiplexer sensors,
  // and so by making it static all sensors can access it.
  private static I2C multiplexer;
  // The actual sensor. All of the methods call this sensor to get the data.
  private ColorSensorV3 sensor;
  // What port on the multiplexer the color sensor is plugged into.
  private final int port;

  /**
   * Create a multiplexed color sensor.
   * 
   * @param i2cPort - What port the multiplexer is plugged into.
   * @param port    - What port the color sensor is plugged into the multiplexer
   *                <br>
   *                (commonly labeled SC3 and SD3 on the PCB, where 3 is the
   *                port)</br>
   */
  public MultiplexedColorSensor(I2C.Port i2cPort, int port) {
    if (multiplexer == null) {
      multiplexer = new I2C(i2cPort, kMultiplexerAddress);
    }
    this.port = port;
    setChannel();
    sensor = new ColorSensorV3(i2cPort);
  }

  /**
   * Helper method. This just sets the multiplexer to the correct port before
   * using the color sensor.
   */
  private void setChannel() {
    multiplexer.write(kMultiplexerAddress, 1 << port);
  }

  /*-----------------------------------------------------------------------*/
  /* Below are all of the methods used for the color sensor. */
  /* All this does is set the channel, then run the command on the sensor. */
  /*-----------------------------------------------------------------------*/

  public Color getColor() {
    setChannel();
    return sensor.getColor();
  }

  public int getProximity() {
    setChannel();
    return sensor.getProximity();
  }

  public RawColor getRawColor() {
    setChannel();
    return sensor.getRawColor();
  }

  public int getRed() {
    setChannel();
    return sensor.getRed();
  }

  public int getGreen() {
    setChannel();
    return sensor.getGreen();
  }

  public int getBlue() {
    setChannel();
    return sensor.getBlue();
  }

  public int getIR() {
    setChannel();
    return sensor.getIR();
  }

  public boolean hasReset() {
    setChannel();
    return sensor.hasReset();
  }
}

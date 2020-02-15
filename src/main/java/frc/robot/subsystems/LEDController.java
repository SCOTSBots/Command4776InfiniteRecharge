/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.ColorShim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Tools.MathTools;
/**
 * We use WS2812b LEDs.
 */
public class LEDController extends SubsystemBase {
  public Burst[] sections;
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;

  /**
   * Creates a new LEDController. This controls the (addressable) LEDs on the robot.
   */
  public LEDController() {
    if (LEDConstants.kHasLEDs) {
      
      m_led = new AddressableLED(LEDConstants.kLEDPWMPort);
      m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
      m_led.setLength(m_ledBuffer.getLength());
      m_led.start();

      sections = new Burst[]{
        //new Chase(0, m_ledBuffer.getLength(), 3, LEDConstants.kRandom)
        new Burst(0, m_ledBuffer.getLength(), 3, ColorShim.kBlack, ColorShim.kRed)
      };
    }
  }
  int m_rainbowFirstPixelHue = 0;
  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
  int m_chase = 0;
  int wait = 1;
  int dt = 4;
  int m_chaseWidth = 5;
  private void chase(int start, int stop, Color first, Color second) {
    wait++;
    if (wait % dt == 0) {
      for(var i = start; i < stop; i++) {
        if ((i + m_chase) % (m_chaseWidth*2) < m_chaseWidth) {
          m_ledBuffer.setLED(i, first);
        }
        else {
          m_ledBuffer.setLED(i, second);
        }
      }
      m_chase += 1;
      m_chase %= m_chaseWidth*2;
    }
  }
  private void solid(int start, int stop, Color color) {
    for(var i = m_ledBuffer.getLength() / 2; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
  }
  private void partial() {
    chase(0, m_ledBuffer.getLength() / 2, Color.kGreen, Color.kYellow);
    chase(m_ledBuffer.getLength() / 2, m_ledBuffer.getLength(), Color.kRed, Color.kBlue);
    //solid(m_ledBuffer.getLength() / 2, m_ledBuffer.getLength(), ColorShim.kOrangeRed);
  }
  private void sections() {
    for (Section s : sections) {
      s.update(m_ledBuffer);
    }
  }
  @Override
  public void periodic() {
    if (LEDConstants.kHasLEDs) {
      sections();
      m_led.setData(m_ledBuffer);
    }
  }
  abstract class Section {
    public int start;
    public int stop;
    abstract void update(AddressableLEDBuffer output);
    public Section(int start, int stop) {
      this.start = start;
      this.stop = stop;
    }
  }

  class Chase extends Section {
    Color[] colors;
    private int dt;
    private int speed;
    private int progress;

    public Chase(int start, int stop, int speed, Color... colors) {
      super(start, stop);
      this.speed = speed;
      this.colors = colors;
    }

    @Override
    void update(AddressableLEDBuffer led) {
      dt++;
      if (dt % speed == 0) {
        for (var i = start; i < stop; i++) {
          double section = colors.length * MathTools.absoluteDecimal(i + progress, Math.abs(stop - start));
          led.setLED(i, colors[(int)section]);
        }
        progress++;
        progress %= Math.abs(stop - start);
      }
    }
  }
  public class Burst extends Section {
    Color baseColor;
    Color disturbedColor;
    private int dt;
    private int speed;
    ArrayList<Integer> disturbances;

    public Burst(int start, int stop, int speed, Color baseColor, Color disturbedColor) {
      super(start, stop);
      this.baseColor = baseColor;
      this.disturbedColor = disturbedColor;
      this.speed = speed;
      disturbances = new ArrayList<Integer>();
    }

    public void clearDisturbances() {
      disturbances.clear();
    }

    @Override
    void update(AddressableLEDBuffer led) {
      dt++;
      if (dt % speed == 0) {
        if (disturbances.size() < 0) { //All good, just use the blank field
          for (var i = start; i < stop; i++) {
            led.setLED(i, baseColor);
          }
        }
        else { //We have disturbances! Add them!
          for (var i = start; i < stop; i++) {
            led.setLED(i, disturbed(i)? disturbedColor:baseColor);
          }
          for(int i = 0; i < disturbances.size(); i++) {
            if (disturbances.get(i) > stop - 2) {
              disturbances.remove(i);
            }
            else {
              disturbances.set(i, 1+disturbances.get(i));
            }
          }
        }
      }
    }
    //Is the current position at a disturbance?
    private boolean disturbed(int position) {
      for (int d : disturbances) {
        if (d == position) return true;
      }
      return false;
    }

    //Add a disturbance point
    public void disturb(int position) {
      disturbances.add(position);
    }
  }
}

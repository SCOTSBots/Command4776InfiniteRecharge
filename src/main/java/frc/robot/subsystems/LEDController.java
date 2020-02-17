/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.ColorShim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.LEDConstants;
import frc.robot.Tools.MathTools;
/**
 * We use WS2812b LEDs.
 */
public class LEDController extends SubsystemBase {
  public Section[] sections;
  public Burst burstInput;
  public ProgressBar bar;
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
      burstInput = new Burst(0, m_ledBuffer.getLength(), 3, ColorShim.kBlack, ColorShim.kRed);
      bar = new ProgressBar(0, m_ledBuffer.getLength(), ColorShim.kBlack, ColorShim.kOrange, 0);
      sections = new Section[]{
        new FMSTimer(0, m_ledBuffer.getLength(), 15, 10, 30, 100)
        //new Chase(0, m_ledBuffer.getLength(), 3, LEDConstants.kRandom)
        //new FMS(0, m_ledBuffer.getLength()),
        //new Chase(m_ledBuffer.getLength()/2, m_ledBuffer.getLength(), 3, LEDConstants.kMainlyGreen)
      };
      //new ProgressBar(0, 2, Color.kAliceBlue, Color.kBlack, 0);
      //double time = DriverStation.getInstance().getMatchTime();
    }
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
  abstract class Section 
  {
    public int start;
    public int stop;
    abstract void update(AddressableLEDBuffer output);
    public Section(int start, int stop) {
      this.start = start;
      this.stop = stop;
    }
    public double percent(int i) {
      return (1.0*i - start) / (1.0*stop - start);
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
  public class Toggle extends Section {
    Color firstColor;
    Color secondColor;
    boolean usingSecond;
    public Toggle(int start, int stop, Color firstColor, Color secondColor) {
      super(start, stop);
      this.firstColor = firstColor;
      this.secondColor = secondColor;
      usingSecond = false;
    }
    @Override
    public void update(AddressableLEDBuffer led) {
      for (var i = start; i < stop; i++) {
        led.setLED(i, usingSecond ? secondColor : firstColor);
      }
    }
    public void setState(boolean usingSecond) {
      this.usingSecond = usingSecond;
    }
    public void toggle() {
      usingSecond = !usingSecond;
    }
  }
  public class ProgressBar extends Section {
    public Color baseColor;
    public Color filledColor;
    double progress;
    DoubleSupplier updatingValue;
    public ProgressBar(int start, int stop) {
      super(start, stop);
    }
    public ProgressBar(int start, int stop, Color baseColor, Color filledColor, double initialProgress) {
      super(start, stop);
      this.baseColor = baseColor;
      this.filledColor = filledColor;
      progress = initialProgress;
      updatingValue = null;
    }
    public ProgressBar(int start, int stop, Color baseColor, Color filledColor, DoubleSupplier updatingValue) {
      super(start, stop);
      this.baseColor = baseColor;
      this.filledColor = filledColor;
      this.updatingValue = updatingValue;
      progress = updatingValue.getAsDouble();
    }

    @Override
    void update(AddressableLEDBuffer output) {
      if (updatingValue != null) { //If we have a supplier of new data, use it!
        progress = updatingValue.getAsDouble();
      }
      progress = MathUtil.clamp(progress, 0.0, 1.0);
      for (var i = start; i < stop; i++) {
        boolean inProgress = (percent(i) < progress); //Check if LED #i is in the progress
        m_ledBuffer.setLED(i, inProgress ? filledColor : baseColor);
      }
    }
    public void setValue(double newProgress){
      progress = newProgress;
    }
    public void setUpdator(DoubleSupplier newUpdator) {
      updatingValue = newUpdator;
    }
  }
  public class FMS extends Section {
    public FMS(int start, int stop) {
      super(start, stop);
    }
    double hue=0;
    @Override
    void update(AddressableLEDBuffer output) {
      smoothFade();
      updateHue();
      int x = (int) (255*fade(percent(0)));
      if (!DriverStation.getInstance().isDisabled())
      System.out.println(x);
      for (var i = start; i < stop; i++) {
        m_ledBuffer.setHSV(i, getHue(i), 255, x);
      }
    }
    void updateHue(){
      hue += speed*180;
      hue %= 1.0;
      if (time++ > delay) {
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        time = 0;
      }
    }
    String gameData = "X";
    int delay = 100;
    int time=0;
    int getHue(int index) {
      if(gameData.length() > 0)
      {
        switch (gameData.charAt(0))
        {
          case 'B' :
            return 120;
          case 'G' :
            return 60;
          case 'R' :
            return 0;
          case 'Y' :
            return 30;
          default :{
              double h = (180.0*hue + (180.0 * percent(index))) % 180;
              return (int)h;
          }
        }
      } else {
        double h = (180.0*hue + (180.0 * percent(index))) % 180;
        return (int)h;
      }
    }
  }
  public class FMSTimer extends ProgressBar {
    final int endGame;
    final int totalAuto;
    final int totalTeleop;
    final Timer timer;

    public FMSTimer(int start, int stop) {
      super(start, stop);
      endGame = 30;
      totalAuto = 15;
      totalTeleop = 135;
      delay = 100;
      timer = new Timer();
      timer.reset();
      baseColor=ColorShim.kBlack;
    }
    public FMSTimer(int start, int stop, int endGame, int totalAuto, int totalTeleop, int delay) {
      super(start, stop);
      this.endGame = endGame;
      this.totalAuto = totalAuto;
      this.totalTeleop = totalTeleop;
      this.delay = delay;
      timer = new Timer();
      timer.reset();
      baseColor=ColorShim.kBlack;
    }
    final int delay;
    int t=0;
    boolean inEndGame = false;
    double lastTime = -1;
    double startEndgame;
    @Override
    public void update(AddressableLEDBuffer output) {
      if (++t > delay) {
        t=0;
        
        boolean auto = DriverStation.getInstance().isAutonomous();
        lastTime = DriverStation.getInstance().getMatchTime();
        if (auto) {
          inEndGame = false;
          filledColor = ColorShim.kYellow;
        }
        else {
          if (lastTime < endGame) {
            inEndGame = true;
            timer.start();
            startEndgame = lastTime;
          }
          else {
            inEndGame = false;
            filledColor = ColorShim.kGreen;
          }
        }
        double setPoint = auto?(lastTime / totalAuto):(lastTime / totalTeleop);
        setValue(setPoint);
      }
      if (inEndGame) {
        double timeLeft = startEndgame - timer.get();
        System.out.println("time left: "+timeLeft);
        if (timeLeft > endGame / 2) {
          filledColor = timeLeft%1>0.5?ColorShim.kRed:ColorShim.kBlack;
        }
        else if (timeLeft > endGame / 4) {
          filledColor = timeLeft%0.5>0.25?ColorShim.kRed:ColorShim.kBlack;
        }
        else {
          filledColor = timeLeft%0.25>0.125?ColorShim.kRed:ColorShim.kBlack;
        }
      }
      if (lastTime <= 0) {
        SUPERAWESOMELIGHTSHOW(start, stop, output);
        return;
      }
      super.update(output);
    }
    
  }
  double speed = 0.0001;
  double fade = 0;
  private void updateFade() {
    fade += speed*255;
    fade %= 1.0;
  }
  boolean down;
  void smoothFade() {
    fade+= down?-0.02:1 * speed * 254;
    if (fade >= 1) {
      fade = 1;
      down = true;
    }
    if (fade <= 0) {
      down = false;
      fade = 0;
    }
  }
  private double fade(double portion) {
    double z = maintain(portion + fade);
    return z*z;
  }
  double maintain(double x) {
    if (x > 0.99 && x < 1.01) {
      return 0.995;
    }
    else if (x > -0.01 && x < 0.01) {
      return 0.005;
    }
    else {
      return x;
    }
  }
  int width = 3;
  public void SUPERAWESOMELIGHTSHOW(int start, int stop, AddressableLEDBuffer led) {
    for(var i = start; i < stop; i++) {
      led.setLED(i, i%(width*2)<width?Color.kRed:Color.kWhite);
    }
  }
}

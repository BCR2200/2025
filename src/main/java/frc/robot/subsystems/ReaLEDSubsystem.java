package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.RobotContainer;

public class ReaLEDSubsystem implements Runnable {
  AddressableLED ledStrip;
  AddressableLEDBuffer buffer;
  Timer timer;

  Strip fullStrip;
  Strip[] strips;

  // ADD MANUAL CORAL INDICATOR

  PowerDistribution pdp;
  ElevClArmSubsystem arm;

  boolean[] conditions;
  int functionIndex = -1;

  Color BetterRed = new Color(75, 0, 0);
  Color BetterBlue = new Color(0, 0, 75);
  Color BetterWhite = Color.kViolet;
  Color allianceColor = BetterRed;

  int sleepInterval = 20;
  int stripIndex = 0;
  int stripIndex2 = 0;

  int[] cursorPositions = { 0, 1, 2 };
  
  boolean modeInit = true;
  public int disabledMode;
  public final int disabledModes = 10;
  SendableChooser<Integer> disableChooser;
  int tempDisabledMode;

  RobotContainer robot;

  public ReaLEDSubsystem() {
    // this.pdp = robot.pdp;
    // this.arm = robot.e;
    // this.robot = robot;


    // disabledMode = (int) (Math.random() * disabledModes);
    disabledMode = (int) (Math.random() * disabledModes);

        /*
        There are 2 vertical strips of LEDs    
        30 left, 30 right                  
                       
                L *|----|* R
         
         */

    fullStrip = new Strip(0, 59);

    strips = new Strip[] {
            new Strip(0, 29), // L
            new Strip(30, 60), // R
    };

    int length = fullStrip.numLEDs;
    buffer = new AddressableLEDBuffer(length);
    ledStrip = new AddressableLED(Constants.LED_STRIP_ID);
    ledStrip.setLength(length);
    ledStrip.start();
    timer = new Timer();
    timer.restart();
    conditions = new boolean[5];

    disableChooser = new SendableChooser<>();
    disableChooser.setDefaultOption("Rise", Integer.valueOf(0));
    disableChooser.addOption("FullStrip", Integer.valueOf(1));

    SmartDashboard.putData(disableChooser);
    // TODO re-enable this if we want LEDs. Disabling to try to speed up robot code.
     new Thread(this, "LED Thread").start();
  }

  private static class Strip {
    // Both start and end are inclusive
    public final int start;
    public final int end;
    public final int direction;
    public final int numLEDs;

    public Strip(int start, int end) {

      this.start = start;
      this.end = end;

      numLEDs = Math.abs(start - end) + 1;

      if (start < end) {
        direction = 1;
      } else {
        direction = -1;
      }
    }
  }

  @Override
  public void run() {
    while (true) {
      synchronized (this) {
        // if (robot.mode == Coral booom or something)
        // ElevClArm, led, enum declaring the different levels
        allianceCheck();
        checkConditions();
        priorityCheck();

        disabledModePicker();
        
        // riseMode(allianceColor, Color.kWhite);
        // setColour(fullStrip, allianceColor);
        // switch (functionIndex) {
          
        // }
        ledStrip.setData(buffer);
      }
      try {
            Thread.sleep(sleepInterval);
        } catch (InterruptedException iex) {
        }
    }
  }

  /** Checks conditions for all LED methods. */
  public void checkConditions() {
    synchronized (this) {
      for (int i = 0; i < conditions.length; i++) {
        conditions[i] = false;
      }
      if (DriverStation.isDisabled()) {
        conditions[4] = true;
      }
      // if (limelight1.resultLength() > 0) {
      // conditions[0] = true;
      // }
    }
  }

  /** Based on the conditions, decides which module to use. */
  public void priorityCheck() {
    synchronized (this) {
      functionIndex = -1;
      for (int i = 0; i < conditions.length; i++) {
        if (conditions[i]) {
          functionIndex = i;
          break;
        }
      }
    }
  }

  public void allianceCheck() {
    // if (Constants.alliance == Alliance.Red) {
    allianceColor = BetterRed;
    // } else {
    //     allianceColor = BetterBlue;
    // }
  }

  /**
   * Clamps the index of the LED to "safely" set the LED to a buffer.
   *
   * @param index The index of the strip.
   * @param color The desired colour of the index.
   */
  public void safeSetLED(int index, Color color) {
    synchronized (this) {
      int clampedIndex = ExtraMath.clamp(index, 0, buffer.getLength() - 1);
      buffer.setLED(clampedIndex, color);
    }
  }

  /**
   * Clamps the index of the LED to "safely" set the LED to a buffer.
   *
   * @param index The index of the strip.
   * @param r The desired red value to set the LED to.
   * @param g The desired green value to set the LED to.
   * @param b The desired blue value to set the LED to.
   */
  public void safeSetLED(int index, double r, double g, double b) {
    synchronized (this) {
      int clampedIndex = ExtraMath.clamp(index, 0, buffer.getLength());
      buffer.setRGB(clampedIndex, (int)(r * 255.0), (int)(g * 255.0), (int)(b * 255.0));
    }
  }

  /**
   * Sets the strip to one static colour.
   *
   * @param strip The strip to display the colour to.
   * @param color The desired colour of the strip.
   */
  public AddressableLEDBuffer setColour(Strip strip, Color color) {
    synchronized (this) {
      for (int i = strip.start; i != strip.end + strip.direction; i += strip.direction) {
        safeSetLED(i, color);
      }
      return buffer;
    }
  }

  /**
   * Given two colours, draws the first to a specific percentage of the strip
   * length, filled in with the 2nd colour.
   *
   * @param strip      The strip to display to.
   * @param percentage The percentage of the colour to display.
   * @param color1     The foreground colour.
   * @param color2     The background colour.
   */
  public void twoColourProgressBar(Strip strip, double percentage, Color color1,
                                   Color color2) {
    synchronized (this) {
      percentage = ExtraMath.clamp(percentage, 0, 1);
      int numLEDs = (int) (strip.numLEDs * percentage);
      setColour(strip, color2);
      for (int i = strip.start; i != numLEDs * strip.direction + strip.start; i += strip.direction) {
        safeSetLED(i, color1);
      }
    }
  }

  /**
   * Draws cursors that vary in position and size depending on the location of
   * notes
   */
  public void followNote() {
    synchronized (this) {
      // setColour(Color.kBlack, buffer, fullStrip);
      // for (int i = 0; i < limelight1.resultLength(); i++) {
      // int size = (int) ExtraMath.rangeMap(limelight1.getTargets()[i].ta, 0, 1,
      // fullStrip.start, fullStrip.end);
      // Color color;
      // if (limelight1.getTargets()[i].className.equals("redbobot")) {
      // color = Color.kRed;
      // } else if (limelight1.getTargets()[i].className.equals("bluebobot")) {
      // color = Color.kBlue;
      // } else if (limelight1.resultLargestAreaTarget() == i) {
      // color = Color.kWhite;
      // } else {
      // color = Color.kOrangeRed;
      // }
      // drawCursor(limelight1.getTargets()[i].tx, -29.8, 29.8, fullStrip,
      // showingBuffer, color, size);
      // }
    }
  }

  /** Gets voltage from the PDP and displays it as a percentage */
  public void displayVoltage() {
    synchronized (this) {
      double voltage = pdp.getVoltage();
      final double minVoltage = 9;
      final double maxVoltage = 12;
      double percentageVoltage = (voltage - minVoltage) / (maxVoltage - minVoltage);
      Color color1 = Color.kGreen;
      Color color2 = Color.kBlack;
      twoColourProgressBar(fullStrip, percentageVoltage, color1, color2);
    }
  }

  /**
   * API for drawing a cursor on the LED strip
   *
   * @param val   The location of the cursor to be placed.
   * @param min   The minimum value of the cursor range.
   * @param max   The maximum value of the cursor range.
   * @param strip The strip to display to.
   * @param color The desired colour of the cursor.
   * @param size  The width of the cursor in number of LEDs.
   */
  public void drawCursor(double val, double min, double max, Strip strip, Color color, int size) {
    synchronized (this) {
      int centerLED = (int) ExtraMath.rangeMap(val, min, max, strip.start, strip.end);
      int halfSize = (int) (size - 1) / 2;
      for (int i = centerLED - halfSize; i <= centerLED + halfSize; i++) {
        safeSetLED(i, color);
      }
    }
  }

  /**
   * Checks if a given index is within the range of a given strip.
   *
   * @param val   The index of the desired LED.
   * @param strip The strip to check the index on.
   * @return Whether or not the index is within the strip range.
   */
  public boolean indexInRange(int val, Strip strip) {
    int min;
    int max;
    if (strip.direction == 1) {
      min = strip.start;
      max = strip.end;
    } else {
      max = strip.start;
      min = strip.end;
    }
    return val >= min && val <= max;
  }


  /** Picks a mode to display while the robot is disabled. */
  public void disabledModePicker() {
    var chosen = disableChooser.getSelected();
    if (chosen == null) {
      chosen = Integer.valueOf(0);
    }
    int pickedDisableMode = chosen.intValue();
    SmartDashboard.putNumber("picked", pickedDisableMode);
    disabledMode = pickedDisableMode;
    if (disabledMode != tempDisabledMode) {
      stripIndex = 0;
      stripIndex2 = 0;
      modeInit = true;
    }
    tempDisabledMode = disabledMode;
    switch (disabledMode) {
      case 0:
        riseMode(allianceColor, BetterWhite);
        break;
      case 1:
        setColour(fullStrip, allianceColor);
        break;
    }
  }

  /**
   * Draws a cursor that follows the LEDs in sequence.
   *
   * @param bg Background colour.
   * @param fg Foreground colour.
   */
  public void cursorMode(Color bg, Color fg) {
    synchronized (this) {
      sleepInterval = 20;
      if (modeInit) {
        cursorPositions = new int[] { 0, 1, 2 };
        modeInit = false;
      }
      setColour(fullStrip, bg);
      for (int i = 0; i < cursorPositions.length; i++) {
        safeSetLED(cursorPositions[i], fg);
        cursorPositions[i]++;
        if (cursorPositions[i] == 60) {
          cursorPositions[i] = 0;
        }
      }
    }
  }

  /**
   * Draws a cursor that rises from the bottom of each strip to the top.
   *
   * @param bg Background colour.
   * @param fg Foreground colour.
   */
  public void riseMode(Color bg, Color fg) {
    synchronized (this) {
      sleepInterval = 30;
      setColour(fullStrip, bg);
      for (int i = 0; i < strips.length; i++) {
        safeSetLED(strips[i].start + strips[i].direction * stripIndex, fg);
      }
      stripIndex++;
      if (stripIndex == strips[0].numLEDs) {
        stripIndex = 0;
      }
    }
  }


  /** Fades in and out the full strip to emulate breathing. */
//   public void breatheMode() {
//     synchronized (this) {
//       sleepInterval = 15;
//       setColour(fullStrip, Color.kBlack);
//       if (allianceColor == BetterRed) {
//         setColour(fullStrip, new Color(stripIndex, 0, 0));
//       } else {
//         setColour(fullStrip, new Color(0, 0, stripIndex));
//       }
//       if (stripIndex >= 75) {
//         breatheDirection = false;
//       } else if (stripIndex <= 0) {
//         breatheDirection = true;
//       }
//       if (breatheDirection) {
//         stripIndex++;
//       } else {
//         stripIndex--;
//       }
//     }
//   }

  /** Blends two colours together by a variable amount. */
  public Color blend(Color color1, Color color2, double blendFactor) {
    blendFactor = ExtraMath.clamp(blendFactor, 0, 1);
    double red = color1.red * blendFactor + color2.red * (1 - blendFactor);
    double green = color1.green * blendFactor + color2.green * (1 - blendFactor);
    double blue = color1.blue * blendFactor + color2.blue * (1 - blendFactor);
    return new Color(red, green, blue);
  }

  /**
   * Flashes between red and blue to create a police siren effect
   *
   * @param color1 A colour to flash.
   * @param color2 A colour to flash.
   */
//   public void sirenMode(Color color1, Color color2) {
//     synchronized (this) {
//       sleepInterval = 100;
//       Color colorA;
//       Color colorB;
//       if (sirenState) {
//         colorA = color2;
//         colorB = color1;
//       } else {
//         colorA = color1;
//         colorB = color2;
//       }
//       for (var strip : halfTopStrips) {
//         setColour(strip, colorA);
//       }
//       for (var strip : halfBotStrips) {
//         setColour(strip, colorB);
//       }
//       sirenState = !sirenState;
//     }
//   }

}
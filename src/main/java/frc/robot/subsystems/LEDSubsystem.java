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

public class LEDSubsystem implements Runnable {
  AddressableLED ledStrip;
  AddressableLEDBuffer buffer;
  Timer timer;

  Strip fullStrip;
  Strip[] strips;
  Strip[] cornerStrips;
  Strip[] halfStrips;
  Strip[] halfTopStrips;
  Strip[] halfBotStrips;
  Strip[] quarter1Strips;
  Strip[] quarter2Strips;
  Strip[] quarter3Strips;
  Strip[] quarter4Strips;
  Strip[] circleStripFront;
  Strip[] circleStripLeft;
  Strip[] circleStripBack;
  Strip[] circleStripRight;
  Strip[][] circleStrips;



  PowerDistribution pdp;
  ElevClArmSubsystem arm;

  boolean[] conditions;
  int functionIndex = -1;

  AnalogInput micInput;
  // double micVal = 5;
  double volumeLow = 5;
  double volumeHigh = 0;

  int[] cursorPositions = { 0, 1, 2 };
  boolean sirenState = false;
  public boolean exciteMode = false;

  public double fakeVUInput = 0.0;

  Color BetterRed = new Color(75, 0, 0);
  Color BetterBlue = new Color(0, 0, 75);
  Color BetterWhite = Color.kViolet;
  Color allianceColor = BetterBlue;

  int sleepInterval = 20;
  int stripIndex = 0;
  int stripIndex2 = 0;
  boolean breatheDirection = true;
  int[] sparkleBrightness = { 0, 0, 0, 0, 0, 0, 0, 0 };
  int[] sparklePosition = { 0, 0, 0, 0, 0, 0, 0, 0 };
  boolean[] sparkleDirection = { true, false, true, false, true, false, true, false };
  public boolean isSignaling = false;
  Color[] cursorTrailFade = { allianceColor, allianceColor, allianceColor, allianceColor };
  boolean modeInit = true;
  public int disabledMode;
  public final int disabledModes = 10;
  SendableChooser<Integer> disableChooser;
  int tempDisabledMode;
  double fakeVUSaved = 0;
  RobotContainer robot;

  public LEDSubsystem(RobotContainer robot) {
    this.pdp = robot.pdp;
    this.arm = robot.e;
    this.robot = robot;


    // disabledMode = (int) (Math.random() * disabledModes);
    disabledMode = (int) (Math.random() * disabledModes);

        /*
        There are 4 vertical strips of LEDs                      
                   *    *
                L *╔----╗* R
                   |    |
                   |    |
                   |    |
                  *╚----╝* 
                   *    *
                  BL    BR

         
         */

    fullStrip = new Strip(0, 87);

    strips = new Strip[] {
            new Strip(0, 10), // FL
            new Strip(21, 11), // LF
            new Strip(22, 32), // LB
            new Strip(43, 33), // BL
            new Strip(44, 54), // BR
            new Strip(65, 55), // RB
            new Strip(66, 76), // RF
            new Strip(87, 77), // FR
    };

    cornerStrips = new Strip[] {
            new Strip(0, 21), // FL + LF
            new Strip(22, 43), // LB + BL
            new Strip(44, 65), // BR + RB
            new Strip(66, 87), // RF + FR
    };

    halfStrips = new Strip[] {
            new Strip(0, 5), // Front Left Bottom
            new Strip(6, 10), // Front Left Top
            new Strip(15, 11), // Left Front Top
            new Strip(21, 16), // Left Front Bottom
            new Strip(22, 27), // Left Back Bottom
            new Strip(28, 32), // Left Back Top
            new Strip(37, 33), // Back Left Top
            new Strip(43, 38), // Back Left Bottom
            new Strip(44, 49), // Back Right Bottom
            new Strip(50, 54), // Back Right Top
            new Strip(59, 55), // Right Back Top
            new Strip(65, 60), // Right Back Bottom
            new Strip(66, 71), // Right Front Bottom
            new Strip(72, 76), // Right Front Top
            new Strip(81, 77), // Front Right Top
            new Strip(87, 82), // Front Right Bottom
    };

    halfTopStrips = new Strip[] {
            new Strip(6, 10), // Front Left Top
            new Strip(15, 11), // Left Front Top
            new Strip(28, 32), // Left Back Top
            new Strip(37, 33), // Back Left Top
            new Strip(50, 54), // Back Right Top
            new Strip(59, 55), // Right Back Top
            new Strip(72, 76), // Right Front Top
            new Strip(81, 77), // Front Right Top
    };

    halfBotStrips = new Strip[] {
            new Strip(0, 5), // Front Left Bottom
            new Strip(21, 16), // Left Front Bottom
            new Strip(22, 27), // Left Back Bottom
            new Strip(43, 38), // Back Left Bottom
            new Strip(44, 49), // Back Right Bottom
            new Strip(65, 60), // Right Back Bottom
            new Strip(66, 71), // Right Front Bottom
            new Strip(87, 82), // Front Right Bottom
    };

    quarter1Strips = new Strip[] {
            new Strip(0, 2),
            new Strip(21, 19),
            new Strip(22, 24),
            new Strip(43, 41),
            new Strip(44, 46),
            new Strip(65, 63),
            new Strip(66, 68),
            new Strip(87, 85),
    };
    quarter2Strips = new Strip[] {
            new Strip(3, 5),
            new Strip(18, 16),
            new Strip(25, 27),
            new Strip(40, 38),
            new Strip(47, 49),
            new Strip(62, 60),
            new Strip(69, 71),
            new Strip(84, 82),
    };
    quarter3Strips = new Strip[] {
            new Strip(6, 8),
            new Strip(15, 13),
            new Strip(28, 30),
            new Strip(37, 35),
            new Strip(50, 52),
            new Strip(59, 57),
            new Strip(72, 74),
            new Strip(81, 79),
    };
    quarter4Strips = new Strip[] {
            new Strip(9, 10),
            new Strip(12, 11),
            new Strip(31, 32),
            new Strip(34, 33),
            new Strip(53, 54),
            new Strip(56, 55),
            new Strip(75, 76),
            new Strip(78, 77),
    };
    circleStripFront = new Strip[]{
            new Strip(0, 10),
            new Strip(77, 87)
    };
    circleStripLeft = new Strip[]{
            new Strip(21, 11),
            new Strip(32, 22)
    };
    circleStripBack = new Strip[]{
            new Strip(43, 33),
            new Strip(54, 44)
    };
    circleStripRight = new Strip[]{
            new Strip(65, 55),
            new Strip(76, 66)
    };
    circleStrips = new Strip[][]{
            circleStripFront,
            circleStripLeft,
            circleStripBack,
            circleStripRight
    };

    int length = fullStrip.numLEDs;
    buffer = new AddressableLEDBuffer(length);
    ledStrip = new AddressableLED(Constants.LED_STRIP_ID);
    ledStrip.setLength(length);
    ledStrip.start();
    timer = new Timer();
    timer.restart();
    conditions = new boolean[5];
    for (int i = 0; i < strips.length; i++) {
      sparkleBrightness[i] = (int) (Math.random() * 76);
    }
    for (int i = 0; i < strips.length; i++) {
      sparklePosition[i] = (int) (Math.random() * 11);
    }

    micInput = new AnalogInput(0);
    micInput.setAverageBits(250);

    disableChooser = new SendableChooser<>();
    disableChooser.setDefaultOption("Cursor", Integer.valueOf(0));
    disableChooser.addOption("Rise", Integer.valueOf(1));
    disableChooser.addOption("Spin", Integer.valueOf(2));
    disableChooser.addOption("Breathe", Integer.valueOf(3));
    disableChooser.addOption("Crackle", Integer.valueOf(4));
    disableChooser.addOption("Sparkle", Integer.valueOf(5));
    disableChooser.addOption("Sparkle2", Integer.valueOf(6));
    disableChooser.addOption("TV Static", Integer.valueOf(7));
    disableChooser.addOption("Siren", Integer.valueOf(8));
    disableChooser.addOption("Snail", Integer.valueOf(9));
    disableChooser.addOption("Circle", Integer.valueOf(10));
    disableChooser.addOption("Real VU", Integer.valueOf(11));
    disableChooser.addOption("Fake VU", Integer.valueOf(12));
    disableChooser.addOption("Fake VU 2", Integer.valueOf(13));
    disableChooser.addOption("Spiral",Integer.valueOf(14));
    disableChooser.addOption("Fire", Integer.valueOf(15));

    SmartDashboard.putData(disableChooser);
    // TODO re-enable this if we want LEDs. Disabling to try to speed up robot code.
    // new Thread(this, "LED Thread").start();
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
        switch (functionIndex) {
          case 0:
            sirenMode(BetterBlue, BetterRed);
            break;
          case 1:
            signalNote();
            break;
          case 2:
            // shooterStatus();
            break;
          case 3:
            hasNote();
            break;
          case 4:
            disabledModePicker();
            // vuMode();
            break;
          default:
            setColour(fullStrip, Color.kBlack);
            // displayVoltage();
            break;
        }
        ledStrip.setData(buffer);
      }
      try {
        if (exciteMode) {
          Thread.sleep(100);
        } else {
          Thread.sleep(sleepInterval);
        }
      } catch (InterruptedException ignored) {
      }
      SmartDashboard.putNumber("fake VU 2 input", fakeVUInput);
    }
  }

  /** Checks conditions for all LED methods. */
  public void checkConditions() {
    synchronized (this) {
      for (int i = 0; i < conditions.length; i++) {
        conditions[i] = false;
      }
      if (exciteMode) {
        conditions[0] = true;
      }
      if (isSignaling) {
        conditions[1] = true;
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

  /** Colours the entire strip orange when a note is visible */
  public void seeingNote() {
    synchronized (this) {
      setColour(fullStrip, Color.kOrangeRed);
    }
  }

  /**
   * Checks whether the current in the shooter rollers exceeds a certain amount,
   * which happens when a note has entered the shooter.
   */
  public void hasNote() {
    setColour(fullStrip, BetterWhite);
  }

  /** Signals to the Human Player to drop a note. */
  public void signalNote(){
    setColour(fullStrip, Color.kOrangeRed);
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
        cursorMode(allianceColor, BetterWhite);
        break;
      case 1:
        riseMode(allianceColor, BetterWhite);
        break;
      case 2:
        spinMode(Color.kBlack, allianceColor);
        break;
      case 3:
        breatheMode();
        break;
      case 4:
        crackleMode(Color.kBlack, allianceColor);
        break;
      case 5:
        sparkleMode();
        break;
      case 6:
        sparkle2Mode();
        break;
      case 7:
        tvStatic();
        break;
      case 8:
        sirenMode(BetterBlue, BetterRed);
        break;
      case 9:
        snailMode(allianceColor, BetterWhite);
        // snailMode(Color.kBlack, allianceColor);
        break;
      case 10:
        circleMode(allianceColor, BetterWhite);
        break;
      case 11:
        realVU();
        break;
      case 12:
        fakeVU();
        break;
      case 13:
        fakeVU2();
        break;
      case 14:
        spiralMode(allianceColor, BetterWhite);
        break;
      case 15:
        fire();
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
        if (cursorPositions[i] == 88) {
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
      sleepInterval = 60;
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

  /**
   * Sets an entire strip to the given colour, looping around all the strips.
   *
   * @param bg Background colour.
   * @param fg Foreground colour.
   */
  public void spinMode(Color bg, Color fg) {
    synchronized (this) {
      sleepInterval = 350;
      setColour(fullStrip, bg);
      setColour(cornerStrips[stripIndex], fg);
      stripIndex++;
      if (stripIndex == cornerStrips.length) {
        stripIndex = 0;
      }
    }
  }

  /** Fades in and out the full strip to emulate breathing. */
  public void breatheMode() {
    synchronized (this) {
      sleepInterval = 15;
      setColour(fullStrip, Color.kBlack);
      if (allianceColor == BetterRed) {
        setColour(fullStrip, new Color(stripIndex, 0, 0));
      } else {
        setColour(fullStrip, new Color(0, 0, stripIndex));
      }
      if (stripIndex >= 75) {
        breatheDirection = false;
      } else if (stripIndex <= 0) {
        breatheDirection = true;
      }
      if (breatheDirection) {
        stripIndex++;
      } else {
        stripIndex--;
      }
    }
  }

  /**
   * Sets one LED to the given colour on each strip at random.
   *
   * @param bg Background colour.
   * @param fg Foreground colour.
   */
  public void crackleMode(Color bg, Color fg) {
    synchronized (this) {
      sleepInterval = 60;
      setColour(fullStrip, bg);
      for (int i = 0; i < strips.length; i++) {
        safeSetLED(strips[i].start + strips[i].direction * (int) (Math.random() * 11), fg);
      }
    }
  }

  /** Fades in and out a LED on each strip to create a sparkling effect. */
  public void sparkleMode() {
    synchronized (this) {
      sleepInterval = 10;
      setColour(fullStrip, Color.kBlack);
      for (int i = 0; i < strips.length; i++) {
        if (sparkleBrightness[i] == 0) {
          sparklePosition[i] = (int) (Math.random() * 11);
        }
        int baseInd = strips[i].start + strips[i].direction * sparklePosition[i];
        if (allianceColor == BetterRed) {
          safeSetLED(baseInd,
                  new Color(sparkleBrightness[i], 0, 0));
          if (indexInRange(baseInd + 1, strips[i])) {
            safeSetLED(baseInd + 1,
                    new Color((int) (sparkleBrightness[i] * 0.1), 0, 0));
          }
          if (indexInRange(baseInd - 1, strips[i])) {
            safeSetLED(baseInd - 1,
                    new Color((int) (sparkleBrightness[i] * 0.1), 0, 0));
          }
        } else {
          safeSetLED(baseInd,
                  new Color(0, 0, sparkleBrightness[i]));
          if (indexInRange(baseInd + 1, strips[i])) {
            safeSetLED(baseInd + 1,
                    new Color(0, 0, (int) (sparkleBrightness[i] * 0.1)));
          }
          if (indexInRange(baseInd - 1, strips[i])) {
            safeSetLED(baseInd - 1,
                    new Color(0, 0, (int) (sparkleBrightness[i] * 0.1)));
          }
        }

        if (sparkleBrightness[i] >= 75) {
          sparkleDirection[i] = false;
        } else if (sparkleBrightness[i] <= 0) {
          sparkleDirection[i] = true;
        }
        if (sparkleDirection[i]) {
          sparkleBrightness[i]++;
        } else {
          sparkleBrightness[i]--;
        }
      }
    }
  }

  /** Fades in and out a LED on each strip to create a sparkling effect. */
  public void sparkle2Mode() {
    synchronized (this) {
      sleepInterval = 20;

      for (int i = 0; i < strips.length; i++) {
        if (sparkleBrightness[i] <= 0) {
          sparklePosition[i] = (int) (Math.random() * 11);
        }

        int baseInd = strips[i].start + strips[i].direction * sparklePosition[i];
        if (allianceColor == BetterRed) {
          setColour(strips[i], new Color(20, 0, 0));

          safeSetLED(baseInd,
                  new Color(sparkleBrightness[i], sparkleBrightness[i], sparkleBrightness[i]));
          if (indexInRange(baseInd + 1, strips[i])) {
            safeSetLED(baseInd + 1,
                    new Color((int) (sparkleBrightness[i] * 0.3), (int) (sparkleBrightness[i] * 0.1),
                            (int) (sparkleBrightness[i] * 0.1)));
          }
          if (indexInRange(baseInd - 1, strips[i])) {
            safeSetLED(baseInd - 1,
                    new Color((int) (sparkleBrightness[i] * 0.3), (int) (sparkleBrightness[i] * 0.1),
                            (int) (sparkleBrightness[i] * 0.1)));
          }
        } else {
          setColour(strips[i], new Color(0, 0, 20));

          safeSetLED(baseInd,
                  new Color(sparkleBrightness[i], sparkleBrightness[i], sparkleBrightness[i]));
          if (indexInRange(baseInd + 1, strips[i])) {
            safeSetLED(baseInd + 1,
                    new Color((int) (sparkleBrightness[i] * 0.1), (int) (sparkleBrightness[i] * 0.1),
                            (int) (sparkleBrightness[i] * 0.3)));
          }
          if (indexInRange(baseInd - 1, strips[i])) {
            safeSetLED(baseInd - 1,
                    new Color((int) (sparkleBrightness[i] * 0.1), (int) (sparkleBrightness[i] * 0.1),
                            (int) (sparkleBrightness[i] * 0.3)));
          }
        }
        if (sparkleBrightness[i] >= 75) {
          sparkleDirection[i] = false;
        } else if (sparkleBrightness[i] <= 0) {
          sparkleDirection[i] = true;
        }
        if (sparkleDirection[i]) {
          sparkleBrightness[i] += 4;
        } else {
          sparkleBrightness[i] -= 4;
        }
      }
    }
  }

  /**
   * Loops a cursor around the full strip with a fading trail behind it.
   *
   * @param bg Background colour.
   * @param fg Foreground colour.
   */
  public void snailMode(Color bg, Color fg) {
    synchronized (this) {
      sleepInterval = 26;
      if (modeInit) {
        cursorPositions = new int[] { 0, 1, 2, 3, 4, 5, 6 };
        cursorTrailFade[0] = blend(fg, bg, 0.05);
        cursorTrailFade[1] = blend(fg, bg, 0.1);
        cursorTrailFade[2] = blend(fg, bg, 0.2);
        cursorTrailFade[3] = blend(fg, bg, 0.3);
        modeInit = false;
      }
      setColour(fullStrip, bg);
      for (int i = 0; i < cursorPositions.length; i++) {
        if (i < 4) {
          safeSetLED(cursorPositions[i], cursorTrailFade[i]);
        } else {
          safeSetLED(cursorPositions[i], fg);
        }
        cursorPositions[i]++;
        if (cursorPositions[i] == 88) {
          cursorPositions[i] = 0;
        }
      }
    }
  }

  public void circleMode(Color bg, Color fg){
    synchronized (this){
      sleepInterval = 20;
      setColour(fullStrip, bg);
      stripIndex = (stripIndex >= circleStripFront[0].numLEDs+circleStripFront[1].numLEDs) ? 0 : stripIndex+1;
      for(var circleStrip : circleStrips){
        Strip curr = (stripIndex < circleStrip[0].numLEDs) ? circleStrip[0]: circleStrip[1];
        safeSetLED(curr.start + (stripIndex-((stripIndex < circleStrip[0].numLEDs) ? 0 : circleStrip[0].numLEDs))*curr.direction, fg);
      }
    }
  }

  public void spiralMode(Color bg, Color fg){
    synchronized (this) {
      sleepInterval = 200;
      setColour(fullStrip, bg);
      safeSetLED(strips[stripIndex2%strips.length].start + strips[stripIndex2%strips.length].direction * stripIndex%strips[0].numLEDs, fg);
      stripIndex++;
      stripIndex2++;
    }
  }

  /** Blends two colours together by a variable amount. */
  public Color blend(Color color1, Color color2, double blendFactor) {
    blendFactor = ExtraMath.clamp(blendFactor, 0, 1);
    double red = color1.red * blendFactor + color2.red * (1 - blendFactor);
    double green = color1.green * blendFactor + color2.green * (1 - blendFactor);
    double blue = color1.blue * blendFactor + color2.blue * (1 - blendFactor);
    return new Color(red, green, blue);
  }

  /** TV static effect. */
  public void tvStatic() {
    synchronized (this) {
      sleepInterval = 20;
      for (int i = 0; i < fullStrip.numLEDs; i++) {
        // 40% LEDS on white, 60% off. Random per LED per iteration
        boolean isBright = Math.random() < 0.4;
        if (isBright) {
          safeSetLED(i, Color.kWhite);
        } else {
          safeSetLED(i, Color.kBlack);
        }
      }
    }
  }

  /**
   * Flashes between red and blue to create a police siren effect
   *
   * @param color1 A colour to flash.
   * @param color2 A colour to flash.
   */
  public void sirenMode(Color color1, Color color2) {
    synchronized (this) {
      sleepInterval = 100;
      Color colorA;
      Color colorB;
      if (sirenState) {
        colorA = color2;
        colorB = color1;
      } else {
        colorA = color1;
        colorB = color2;
      }
      for (var strip : halfTopStrips) {
        setColour(strip, colorA);
      }
      for (var strip : halfBotStrips) {
        setColour(strip, colorB);
      }
      sirenState = !sirenState;
    }
  }

  public void fakeVU() {
    synchronized (this) {
      sleepInterval = 20;
      for (int i = 0; i < strips.length; i++) {
        setColour(strips[i], Color.kBlack);
        //double micVal = (int) ExtraMath.rangeMap(micInput.getAverageVoltage(), volumeLow, volumeHigh, 0, 11.9);
        double interval = (((int)(Math.random()*3))-1)*0.3;
        fakeVUSaved = fakeVUSaved + interval;
        fakeVUSaved = ExtraMath.clamp(fakeVUSaved, 0, 11.9);
        for (int j = strips[i].start; j != (int) fakeVUSaved * strips[i].direction
                + strips[i].start; j += strips[i].direction) {
          if (ExtraMath.within(j, strips[i].start, 11)) {
            safeSetLED(j, Color.kRed);
          }
          if (ExtraMath.within(j, strips[i].start, 8)) {
            safeSetLED(j, Color.kGold);
          }
          if (ExtraMath.within(j, strips[i].start, 6)) {
            safeSetLED(j, Color.kGreen);
          }
        }
        // SmartDashboard.putNumber("Mic Input", micInput.getAverageVoltage());
      }
    }
  }

  public void fakeVU2() {
    synchronized (this) {
      double micVal = (int) ExtraMath.rangeMap(fakeVUInput, 0, 1, 0, 11.9);
      sleepInterval = 20;
      micVal = ExtraMath.clamp(micVal, 2, 11.9);
      SmartDashboard.putNumber("fakevu2 mic val", micVal);
      for (var strip : strips) {
        setColour(strip, Color.kBlack);
        for (int j = strip.start; j != (int) micVal * strip.direction
                + strip.start; j += strip.direction) {
          if (ExtraMath.within(j, strip.start, 11)) {
            safeSetLED(j, Color.kRed);
          }
          if (ExtraMath.within(j, strip.start, 8)) {
            safeSetLED(j, Color.kGold);
          }
          if (ExtraMath.within(j, strip.start, 6)) {
            safeSetLED(j, Color.kGreen);
          }
        }
      }
    }
  }

  /** Displays a VU Meter to bounce along with the music. */
  public void realVU() {
    synchronized (this) {
      sleepInterval = 20;
      for (int i = 0; i < strips.length; i++) {
        setColour(strips[i], Color.kBlack);
        if (micInput.getAverageVoltage() < volumeLow) {
          volumeLow = micInput.getAverageVoltage();
        }
        if (micInput.getAverageVoltage() > volumeHigh) {
          volumeHigh = micInput.getAverageVoltage();
        }
        volumeLow += 0.001;
        volumeHigh -= 0.001;
        double micVal = (int) ExtraMath.rangeMap(micInput.getAverageVoltage(), volumeLow, volumeHigh, 0, 11.9);
        // double interval = (((int)(Math.random()*3))-1)*0.3;
        // micVal = micVal + interval;
        micVal = ExtraMath.clamp(micVal, 0, 11.9);
        for (int j = strips[i].start; j != (int) micVal * strips[i].direction
                + strips[i].start; j += strips[i].direction) {
          if (ExtraMath.within(j, strips[i].start, 11)) {
            safeSetLED(j, Color.kRed);
          }
          if (ExtraMath.within(j, strips[i].start, 8)) {
            safeSetLED(j, Color.kGold);
          }
          if (ExtraMath.within(j, strips[i].start, 6)) {
            safeSetLED(j, Color.kGreen);
          }
        }
        // SmartDashboard.putNumber("Mic Input", micInput.getAverageVoltage());
      }
    }
  }

  public void fire() {
    synchronized (this) {
      sleepInterval = 20;
      // desaturated alliance color looks cooler imo lol
      double desaturated_red = allianceColor.red / 2.0 + 0.5;
      double desaturated_green = allianceColor.green / 2.0 + 0.5;
      double desaturated_blue = allianceColor.blue / 2.0 + 0.5;
      for (int i = 0; i < fullStrip.numLEDs; i++) {
        // get "random" frequency
        double frequency = ExtraMath.hashPrand(i, 1.0, 2.0);
        // downwards sawtooth oscillation
        // use current timer value and restrict to a range from 0-1,
        // then invert to make it ramp down from 1 to 0
        double period = 1.0 - (timer.get() * frequency) % 1.0;
        // use the period to calculate the brightness of this LED's color, ranging from 1 to 0.5.
        double colorMult = period / 2.0 + 0.5;
        // SET THE LED
        safeSetLED(i, new Color(
                desaturated_red  * colorMult,
                desaturated_green * colorMult,
                desaturated_blue * colorMult
        ));
      }
    }
  }

}
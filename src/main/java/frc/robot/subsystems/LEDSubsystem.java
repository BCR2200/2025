package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.ExtraMath;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.led.LEDDrawer;
import frc.robot.subsystems.led.Strip;
import frc.robot.subsystems.led.disabledmodes.Full;
import frc.robot.subsystems.led.disabledmodes.Rise;
import frc.robot.subsystems.led.disabledmodes.Sink;
import frc.robot.subsystems.led.disabledmodes.TestColors;
import frc.robot.subsystems.led.disabledmodes.Breathe;
import frc.robot.subsystems.led.disabledmodes.Siren;
import frc.robot.subsystems.led.disabledmodes.Bounce;

public class LEDSubsystem implements Runnable {
  AddressableLED ledStrip;
  AddressableLEDBuffer buffer;
  Timer timer;
  Timer algaeTimer;

  public Strip fullStrip;
  public Strip[] strips;
  public Strip[] halfTopStrips;
  public Strip[] halfBotStrips;

  // ADD MANUAL CORAL INDICATOR

  PowerDistribution pdp;
  ElevClArmSubsystem arm;

  public Color BetterRed = new Color(75, 0, 0);
  public Color BetterBlue = new Color(0, 0, 75);
  Color BetterWhite = Color.kViolet;
  public Color allianceColor = BetterRed;

  public int sleepInterval = 20;

  int[] cursorPositions = { 0, 1, 2 };

  boolean modeInit = true;
  SendableChooser<LEDDrawer> disableChooser;

  RobotContainer robot;

  public LEDSubsystem(RobotContainer robot) {
    this.pdp = robot.pdp;
    this.arm = robot.e;
    this.robot = robot;

    /*
     * There are 2 vertical strips of LEDs
     * 30 left, 30 right
     * 
     * L *|----|* R
     * 
     */

    fullStrip = new Strip(0, 59);

    halfTopStrips = new Strip[] {
      new Strip(0, 14), // Bottom L
      new Strip(45, 59), // Bottom R
    };

    halfBotStrips = new Strip[] {
      new Strip(15, 29), // Top L
      new Strip(30, 44), // Top R
    };

    strips = new Strip[] {
        new Strip(0, 29), // L
        new Strip(30, 59), // R
    };

    int length = fullStrip.numLEDs;
    buffer = new AddressableLEDBuffer(length);
    ledStrip = new AddressableLED(Constants.LED_STRIP_ID);
    ledStrip.setLength(length);
    ledStrip.start();
    timer = new Timer();
    algaeTimer = new Timer();
    timer.restart();
    algaeTimer.restart();

    disableChooser = new SendableChooser<>();
    disableChooser.setDefaultOption("Rise", new Rise(this, ledStrip, buffer, BetterWhite));
    disableChooser.addOption("Full", new Full(this, ledStrip, buffer));
    disableChooser.addOption("Sink", new Sink(this, ledStrip, buffer, BetterWhite));
    disableChooser.addOption("TestColors", new TestColors(this, ledStrip, buffer, BetterWhite)); //This exists to see what the colors are, without having to enable every time
    disableChooser.addOption("Breathe", new Breathe(this, ledStrip, buffer)); 
    disableChooser.addOption("Siren", new Siren(this, ledStrip, buffer));
    disableChooser.addOption("Bounce", new Bounce(this, ledStrip, buffer, BetterWhite)); // Like rise and sink, but goes back down after hitting the top and vice versa

    SmartDashboard.putData(disableChooser);
    new Thread(this, "LED Thread").start();
  }

  @Override
  public void run() {
    while (true) {
      synchronized (this) {
        allianceCheck();
        if (DriverStation.isEnabled()) {
          enabledMode();
        }
        if (DriverStation.isDisabled()) {
          disabledMode();
        }
        // riseMode(allianceColor, Color.kWhite);
        // setColour(fullStrip, allianceColor);
        // switch (functionIndex) {

        // }
        ledStrip.setData(buffer);
      }
      try {
        Thread.sleep(sleepInterval);
      } catch (InterruptedException ignored) {
      }
    }
  }

  public void allianceCheck() {
    if (Robot.alliance == Alliance.Red) {
      allianceColor = BetterRed;
    } else {
      allianceColor = BetterBlue;
    }
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
   * @param r     The desired red value to set the LED to.
   * @param g     The desired green value to set the LED to.
   * @param b     The desired blue value to set the LED to.
   */
  public void safeSetLED(int index, double r, double g, double b) {
    synchronized (this) {
      int clampedIndex = ExtraMath.clamp(index, 0, buffer.getLength());
      buffer.setRGB(clampedIndex, (int) (r * 255.0), (int) (g * 255.0), (int) (b * 255.0));
    }
  }

  /**
   * Sets the strip to one static colour.
   *
   * @param strip The strip to display the colour to.
   * @param color The desired colour of the strip.
   */
  public void setColour(Strip strip, Color color) {
    synchronized (this) {
      for (int i = strip.start; i != strip.end + strip.direction; i += strip.direction) {
        safeSetLED(i, color);
      }
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

  /* Displays the mode, as well as if there's a coral/algae in the claw/hopper */
  public void enabledMode() {

    // Gather data
    ControlMode mode = arm.getEMode();
    double current = arm.clawMotor.getCurrent();
    boolean coralInHopper = arm.isCoralInHopper();
    boolean coralInClaw = arm.isCoralInClaw();
    double algaeTime = algaeTimer.get();

    // Change coral hopper coral (alliance or a unique color)
    Color coralColor = Color.kPurple;
    //Color coralColor = allianceColor
    
    // Display mode
    Color algaeColor = Color.kGreen;
    Color climbColor = Color.kDarkOrange;
    if (mode == ControlMode.Coral && !coralInHopper && !coralInClaw) {
      setColour(fullStrip, coralColor);
    }
    if (mode == ControlMode.Algae && current < 20) {
      setColour(fullStrip, algaeColor);
      if (algaeTime != 0) {
        algaeTimer.reset();
      }
    }
    if (mode == ControlMode.Climb) {
      setColour(fullStrip, climbColor);
    }

    // Display if algae is in claw for a short time
    Color colorAlgaeClawWithTime = Color.kSeaGreen;
    if (mode == ControlMode.Algae && current >= 20 && algaeTime < 15) {
      setColour(fullStrip, colorAlgaeClawWithTime);
      if (algaeTime == 0) {
        algaeTimer.start();
      }
    }

    // Display if algae is in claw and it's been there for over 15 secs (hurting the motors)
    Color colorAlgaeClawWithoutTime = Color.kLimeGreen;
    if (mode == ControlMode.Algae && current >= 20 && algaeTime >= 15) {
      setColour(fullStrip, colorAlgaeClawWithoutTime);
    }

    // Display if coral is in hopper
    Color colorCoralHopper = Color.kMediumVioletRed;
    if (mode == ControlMode.Coral && coralInHopper) {
      setColour(fullStrip, colorCoralHopper);
    }

    // Display if coral is in claw
    Color colorCoralClaw = Color.kDeepPink;
    if (mode == ControlMode.Coral && coralInClaw && !coralInHopper) { //Coral not in hopper to make hopper colour longer
      setColour(fullStrip, colorCoralClaw);
    }
  }

  /* These were the individual functions before they were combined into enabledMode
  public void algaeClaw() {
  synchronized (this) {
  ControlMode mode = arm.getEMode();
  double current = arm.clawMotor.getCurrent();
  Color color1 = Color.kSeaGreen;
  Color color2 = Color.kLimeGreen;
  if (mode == ControlMode.Algae && current > 20) {
  twoColourProgressBar(fullStrip, 50, color1, color2);
  }}}

  public void coralHopper() {
  synchronized (this) {
  ControlMode mode = arm.getEMode();
  Boolean coralInHopper = arm.isCoralInHopper();
  Color color = Color.kMediumVioletRed;
  if (mode == ControlMode.Coral && coralInHopper == true) {
  setColour(fullStrip, color);
  }}}

  public void coralClaw() {
  synchronized (this) {
  ControlMode mode = arm.getEMode();
  Boolean coralInClaw = arm.isCoralInClaw();
  Color color1 = Color.kLightPink;
  Color color2 = Color.kDeepPink;
  if (mode == ControlMode.Coral && coralInClaw == true) {
  twoColourProgressBar(fullStrip, 50, color1, color2);
  }}}
  */

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
  public void disabledMode() {
    var chosen = disableChooser.getSelected();
    if (chosen == null) {
      return;
    }
    chosen.draw();
    sleepInterval = chosen.sleepInterval();
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

  /** Fades in and out the full strip to emulate breathing. */
  // public void breatheMode() {
  // synchronized (this) {
  // sleepInterval = 15;
  // setColour(fullStrip, Color.kBlack);
  // if (allianceColor == BetterRed) {
  // setColour(fullStrip, new Color(stripIndex, 0, 0));
  // } else {
  // setColour(fullStrip, new Color(0, 0, stripIndex));
  // }
  // if (stripIndex >= 75) {
  // breatheDirection = false;
  // } else if (stripIndex <= 0) {
  // breatheDirection = true;
  // }
  // if (breatheDirection) {
  // stripIndex++;
  // } else {
  // stripIndex--;
  // }
  // }
  // }

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
  // public void sirenMode(Color color1, Color color2) {
  // synchronized (this) {
  // sleepInterval = 100;
  // Color colorA;
  // Color colorB;
  // if (sirenState) {
  // colorA = color2;
  // colorB = color1;
  // } else {
  // colorA = color1;
  // colorB = color2;
  // }
  // for (var strip : halfTopStrips) {
  // setColour(strip, colorA);
  // }
  // for (var strip : halfBotStrips) {
  // setColour(strip, colorB);
  // }
  // sirenState = !sirenState;
  // }
  // }

}
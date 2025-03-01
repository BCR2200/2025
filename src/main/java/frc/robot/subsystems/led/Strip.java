package frc.robot.subsystems.led;

public class Strip {
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
package frc.robot.input;

public class DPadButtonTest {

  public static void main(String[] args) {
    DPadButtonTest test = new DPadButtonTest();
    test.testGetAsBoolean_TableDrivenWithDetailedCases();
    System.out.println("All tests passed!");
  }

  private static class TestXboxController implements DPadButton.POVSupplier {
    private int pov;

    public TestXboxController() {
    }

    public void setPOV(int pov) {
      this.pov = pov;
    }

    @Override
    public int getPOV() {
      return this.pov;
    }
  }

  // Simple assertion method
  private static void assertEquals(Object expected, Object actual, String message) {
    if (expected == null && actual == null) {
      return;
    }
    if (expected != null && expected.equals(actual)) {
      return;
    }
    throw new AssertionError(message + ": Expected " + expected + " but got " + actual);
  }

  public void testGetAsBoolean_TableDrivenWithDetailedCases() {
    // Test data in the format {DPad, POV value, expected result}
    Object[][] testCases = {
            // Up: Valid at 0 (directly up), 45 (upper edge), and 315 (lower edge)
            {DPadButton.DPad.Up, 0, true},
            {DPadButton.DPad.Up, 45, true},
            {DPadButton.DPad.Up, 315, true},
            {DPadButton.DPad.Up, 90, false},
            {DPadButton.DPad.Up, 135, false},
            {DPadButton.DPad.Up, 180, false},
            {DPadButton.DPad.Up, 225, false},
            {DPadButton.DPad.Up, 270, false},

            // Down: Valid at 180 (directly down), 135 (upper edge), and 225 (lower edge)
            {DPadButton.DPad.Down, 180, true},
            {DPadButton.DPad.Down, 135, true},
            {DPadButton.DPad.Down, 225, true},
            {DPadButton.DPad.Down, 0, false},
            {DPadButton.DPad.Down, 45, false},
            {DPadButton.DPad.Down, 90, false},
            {DPadButton.DPad.Down, 270, false},
            {DPadButton.DPad.Down, 315, false},

            // Left: Valid at 270 (directly left), 225 (upper edge), and 315 (lower edge)
            {DPadButton.DPad.Left, 270, true},
            {DPadButton.DPad.Left, 225, true},
            {DPadButton.DPad.Left, 315, true},
            {DPadButton.DPad.Left, 0, false},
            {DPadButton.DPad.Left, 45, false},
            {DPadButton.DPad.Left, 90, false},
            {DPadButton.DPad.Left, 135, false},
            {DPadButton.DPad.Left, 180, false},

            // Right: Valid at 90 (directly right), 45 (upper edge), and 135 (lower edge)
            {DPadButton.DPad.Right, 90, true},
            {DPadButton.DPad.Right, 45, true},
            {DPadButton.DPad.Right, 135, true},
            {DPadButton.DPad.Right, 0, false},
            {DPadButton.DPad.Right, 180, false},
            {DPadButton.DPad.Right, 225, false},
            {DPadButton.DPad.Right, 270, false},
            {DPadButton.DPad.Right, 315, false},

            // OnlyUp: Valid ONLY at 0 (directly up)
            {DPadButton.DPad.OnlyUp, 0, true},
            {DPadButton.DPad.OnlyUp, 45, false},  // Edge case
            {DPadButton.DPad.OnlyUp, 315, false}, // Edge case
            {DPadButton.DPad.OnlyUp, 90, false},
            {DPadButton.DPad.OnlyUp, 180, false},
            {DPadButton.DPad.OnlyUp, 270, false},

            // OnlyDown: Valid ONLY at 180 (directly down)
            {DPadButton.DPad.OnlyDown, 180, true},
            {DPadButton.DPad.OnlyDown, 135, false},  // Edge case
            {DPadButton.DPad.OnlyDown, 225, false},  // Edge case
            {DPadButton.DPad.OnlyDown, 0, false},
            {DPadButton.DPad.OnlyDown, 90, false},
            {DPadButton.DPad.OnlyDown, 270, false},

            // OnlyLeft: Valid ONLY at 270 (directly left)
            {DPadButton.DPad.OnlyLeft, 270, true},
            {DPadButton.DPad.OnlyLeft, 225, false},  // Edge case
            {DPadButton.DPad.OnlyLeft, 315, false},  // Edge case
            {DPadButton.DPad.OnlyLeft, 0, false},
            {DPadButton.DPad.OnlyLeft, 90, false},
            {DPadButton.DPad.OnlyLeft, 180, false},

            // OnlyRight: Valid ONLY at 90 (directly right)
            {DPadButton.DPad.OnlyRight, 90, true},
            {DPadButton.DPad.OnlyRight, 45, false},  // Edge case
            {DPadButton.DPad.OnlyRight, 135, false}, // Edge case
            {DPadButton.DPad.OnlyRight, 0, false},
            {DPadButton.DPad.OnlyRight, 180, false},
            {DPadButton.DPad.OnlyRight, 270, false}
    };

    TestXboxController controller = new TestXboxController();

    for (Object[] testCase : testCases) {
      DPadButton.DPad dPad = (DPadButton.DPad) testCase[0];
      int pov = (int) testCase[1];
      boolean expected = (boolean) testCase[2];

      controller.setPOV(pov);
      DPadButton dPadButton = new DPadButton(controller, dPad);
      assertEquals(expected, dPadButton.getAsBoolean(),
              "Failed for DPad: " + dPad + ", POV: " + pov);
    }
  }
}
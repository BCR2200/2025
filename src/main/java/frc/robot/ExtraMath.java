package frc.robot;

public final class ExtraMath {
    private ExtraMath() {}

    /**
     * Forces a value into a set range, saturating at the bounds.
     * 
     * For more info: https://www.desmos.com/calculator/kfyws5ap6u
     * @param val The value to clamp.
     * @param start The minimum value.
     * @param end The maximum value.
     * @return The clamped value.
     */
    public static double clamp(double val, double start, double end) {
        double min = Math.min(start, end);
        double max = Math.max(start, end);
        if (val > max) {
            val = max;
        } else if (val < min) {
            val = min;
        }
        return val;
    }

    /**
     * Forces a value into a set range, saturating at the bounds.
     * 
     * For more info: https://www.desmos.com/calculator/kfyws5ap6u
     * @param val The value to clamp.
     * @param start The minimum value.
     * @param end The maximum value.
     * @return The clamped value.
     */
    public static int clamp(int val, int start, int end) {
        int min = Math.min(start, end);
        int max = Math.max(start, end);
        if (val > max) {
            val = max;
        } else if (val < min) {
            val = min;
        }
        return val;
    }

    /**
     * Maps a value from one range to another.
     * 
     * For more info: https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another |
     * https://www.desmos.com/calculator/0zisstnbzz
     * @param val The value within `low1..high1` to convert to the range of `low2..high2`.
     * @param low1 The low point of the range of `value`.
     * @param high1 The high point of the range of `value`.
     * @param low2 The low point of the range to convert to.
     * @param high2 The high point of the range to convert to.
     * @return The mapped value.
     */
    public static double rangeMap(double val, double low1, double high1, double low2, double high2) {
        return ((high2 - low2) / (high1 - low1)) * (val - low1) + low2;
    }

    /**
     * Checks if the difference between two values is within a given range. `|(a-b)|<=epsilon`.
     * 
     * For more information: https://www.desmos.com/calculator/ljmvpzakwx
     * @param a The first value to check.
     * @param b The second value to check.
     * @param epsilon The maximum difference. The range of accepted values is `-epsilon..=epsilon`.
     * @return Whether or not a and b are within `epsilon` of each other.
     */
    public static boolean within(double a, double b, double epsilon) {
        return Math.abs(a - b) <= epsilon;
    }

    /**
     * Checks if two values are within a certain factor of each other. In other words, checks if a is `factor` times less than b, and vice versa.
     * 
     * For example, if `a = 1`, `b = 2`, and `factor = 3`, this function returns `true`. b is less than 3 times a.
     * 
     * For more information: https://www.desmos.com/calculator/31aityouvf
     * @param a The first value to check.
     * @param b The last value to check.
     * @param factor The factor threshold to check against.
     * @return Whether or not a and b are proportionally close within the given factor.
     */
    public static boolean withinFactor(double a, double b, double factor) {
        // If the signs of the factor match the signs of a and b, this is true.
        // Rearranged, if the sign of a is the sign of factor * b, this is true.
        boolean signsOk = (Math.signum(factor) > 0 == Math.signum(a * b) > 0);
        a = Math.abs(a);
        b = Math.abs(b);
        // Bound checks fail if |factor|<1, but logic remains when flipping the proportions.
        if (Math.abs(factor) < 1) {
            factor = 1.0 / factor;
        }
        return signsOk && (a / b <= factor && b / a <= factor);
    }

    /**
     * Calculates the Euclidean remainder for l % r.
     * 
     * For more information: https://www.desmos.com/calculator/xo5vuxqdm0
     * @param l The value to take the remainder of.
     * @param r The divisor.
     * @return Euclidean l % r.
     */
    public static double remEuclid(double l, double r) {
        double m = l % r;
        return m + (m < 0 ? Math.abs(r) : 0);
    }

    /**
     * Calculates the minimum distance needed to get from the angle `current` to the angle `target`.
     * 
     * For more information: https://www.desmos.com/calculator/wcyxsf85px
     * @param current The current angle.
     * @param target The target angle.
     * @return The angle to be added to `current` to get to `target`.
     */
    public static double degreeDistance(double current, double target) {
        current = remEuclid(current, 360);
        target = remEuclid(target, 360);
        double dist = target - current;
        if (dist < -180) return dist + 360;
        if (dist > 180) return dist - 360;
        return dist;
    }

    /**
     * Calculates the average (mean) for the given values.
     * 
     * For more information: https://www.desmos.com/calculator/rtmkoyfyrh
     * @param values The values to average together.
     * @return The mean of `values`.
     */
    public static double average(double... values) {
        double sum = 0;
        for(int i = 0; i < values.length; i++){
            sum += values[i];
        }
        return sum/values.length;
    }

    /**
     * Naive deadzone.
     * @param val The axis value.
     * @param deadzone The deadzone threshold.
     * @return 0 if val is within -deadzone..deadzone, otherwise val.
     */
    public static double naiveDeadzone(double val, double deadzone) {
        if (Math.abs(val) < Math.abs(deadzone)) {
            return 0.0;
        } else {
            return val;
        }
    }
    /**
     * Smarter deadzone.
     * @param val
     * @param deadzone
     * @return 1 when val is 1, -1 when val is -1, 0 if deadzone
     */
    public static double deadzone(double val, double deadzone) {
        if (Math.abs(val) < Math.abs(deadzone)) {
            return 0.0;
        } else {
            return rangeMap(Math.abs(val), Math.abs(deadzone), 1, 0, 1) * Math.signum(val);//(val - Math.signum(val) * deadzone) / (1.0 - deadzone);
        }
    }
    
    /**
     * A pseudo random function that acts as a hashing function (VERY NOT CRYPTOGRAPHICALLY SECURE).
     * Generates a seemingly random number.
     * Identical inputs give identical outputs.
     * @param seed The seed for the random generation.
     * @param min The minimum number that can be returned.
     * @param max THe maximum number that can be returned.
     * @return A pseudo-random number.
     */
    public static double hashPrand(double seed, double min, double max) {
        // polynomial equation into a sine function
        // massive frequency + massive amplitude + irrational period (sin is based on pi) = random enough
        double rand = (Math.sin(seed * seed * 277347493292.34562 + seed * 727739942.4878 + 283.2) * 12345.0);
        return (rand * max) % (max - min) + min;
    }
}

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum ReefSide {
    FL, FC, FR, BL, BC, BR;

    public int getTag() {
        boolean isOnRed = Robot.alliance == Alliance.Red;
        return switch(this) {
            case BC -> isOnRed ? 10 : 21;
            case BL -> isOnRed ? 11 : 20;
            case BR -> isOnRed ? 9 : 22;
            case FC -> isOnRed ? 7 : 18;
            case FL -> isOnRed ? 6 : 19;
            case FR -> isOnRed ? 8 : 17;
        };
    }
}

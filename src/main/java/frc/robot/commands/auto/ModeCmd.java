package frc.robot.commands.auto;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;

public class ModeCmd extends Command {
    private final ElevClArmSubsystem e;
    private final ControlMode mode;
    boolean finished;

    public ModeCmd(ElevClArmSubsystem e, ControlMode mode) {
        this.e = e;
        this.mode = mode;
        // do not require, sets position to none lol
    }

    @Override
    public void initialize() {
        finished = false;
        e.requestMode(mode);
    }

    @Override
    public void execute() {
        finished = (e.getEMode() == mode);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

}

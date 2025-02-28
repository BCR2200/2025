package frc.robot.commands.auto;

import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeModeCmd extends Command {
    private final ElevClArmSubsystem e;
    boolean finished;

    public AlgaeModeCmd(ElevClArmSubsystem e) {
        this.e = e;
        // do not require, sets position to none lol
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        e.requestMode(ControlMode.Algae);
        finished = (e.getEMode() == ControlMode.Algae);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

}

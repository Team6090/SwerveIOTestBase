package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RunnableCommand extends CommandBase {
    Runnable run;
    boolean isFinished = false;
    public RunnableCommand(Runnable run, SubsystemBase... requiredSubsystems) {
        addRequirements(requiredSubsystems);
        this.run = run;
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        run.run();
        isFinished = true;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
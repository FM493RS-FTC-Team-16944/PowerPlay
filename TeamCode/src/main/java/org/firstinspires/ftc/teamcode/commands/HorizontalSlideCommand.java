package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;

public class HorizontalSlideCommand extends CommandBase {
    private final LiftSubsystem subsystem;
    private final double position;

    public HorizontalSlideCommand(LiftSubsystem subsystem, double position) {
        this.subsystem = subsystem;
        this.position = position;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return subsystem.horizontalSlide.getCurrentPosition() <= position + 10 &&
                subsystem.horizontalSlide.getCurrentPosition() >= position - 10;
    }
}

package org.firstinspires.ftc.teamcode.models;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class ExtendOutMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;

    public ExtendOutMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;
    }

    @Override
    public void run() {
        robot.intake.groundIntake(state.armPos);
        robot.intake.openClaw();
        robot.lift.setHorizontalSlide(state.horizontalPos);
    }
}

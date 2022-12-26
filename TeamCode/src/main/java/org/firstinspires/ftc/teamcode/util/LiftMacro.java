package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

public class LiftMacro implements Runnable {
    private SampleMecanumDrive robot;

    int EXTEND_LIFT_POSITION = 0;

    double HORIZONTAL_LIFT_POWER = 0.00;
    double VERTICAL_LIFT_POWER = 0.00;

    public LiftMacro(SampleMecanumDrive robot) {
        this.robot = robot;
    }

    @Override
    public void run() {
        this.robot.setHorizontalLift(EXTEND_LIFT_POSITION, VERTICAL_LIFT_POWER);


    }
}

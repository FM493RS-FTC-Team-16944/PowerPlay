package org.firstinspires.ftc.teamcode.models;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class TransferMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;

    public TransferMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;
    }

    public void run() {
        this.robot.intake.hangingIntake();
        this.robot.lift.setHorizontalSlide(ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION);

        this.robot.intake.rotatedHangingIntake();

    }
}

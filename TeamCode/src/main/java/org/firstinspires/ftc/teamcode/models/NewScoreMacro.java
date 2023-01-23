package org.firstinspires.ftc.teamcode.models;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class NewScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;
    private final Thread extendOutRunnable;
    private final Thread transferRunnable;
    private final Telemetry telemetry;

    public boolean finished = false;

    public NewScoreMacro(MecanumDrive robot, GrabPosition state, Telemetry telemetry) {
        this.robot = robot;
        this.state = state;

        this.telemetry = telemetry;

        this.extendOutRunnable = new Thread(
                new ExtendOutMacro(robot, state)
        );
        this.transferRunnable = new Thread(
                new TransferMacro(robot, state)
        );
    }

    public void run() {
        extendOutRunnable.start();

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10) {
                break;
            }
        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.robot.intake.closeClaw();

        this.transferRunnable.start();

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= 10 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= -10)
                break;
        }

        this.robot.intake.openClaw();
        this.robot.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION + 10 &&
                    robot.lift.getVerticalLiftPosition() >= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - 10)
                break;
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= 3 &&
                    robot.lift.getVerticalLiftPosition() >= -3)
                break;
        }

        this.finished = true;
    }
}

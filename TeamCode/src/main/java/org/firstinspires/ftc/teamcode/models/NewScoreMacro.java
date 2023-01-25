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

        // rather than sleeping it might be better to check the position of the claw, actually idek why this is here
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.robot.intake.closeClaw();

        // same with this sleep
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.transferRunnable.start();

        // NOTE: it could be edging the platform because the error margin is 10 ticks, but idk 10 ticks isnt a lot
        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= 10 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= -10)
                break;
        }

        // NOTE: this can be optimized, transfer intake can happen while the horizontal slide is moving
        // this would remove the overhead of sleeping half a second and rather can open the claw instantly once the horizontal slide is at "0"
        this.robot.intake.transferIntake();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.robot.intake.openClaw();
        this.robot.intake.groundIntake(0);
        this.robot.lift.setHorizontalSlide(1300);

        // if the arm is positioned to perfectly place the cone on the platform, this sleep can be removed
        try {
            Thread.sleep(700);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        this.robot.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION + 10 &&
                    robot.lift.getVerticalLiftPosition() >= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - 10)
                break;
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

        // this isn't really necessary until the very last cone, because it should be lowering while queueing up the next cone
        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= 1 &&
                    robot.lift.getVerticalLiftPosition() >= -1)
                break;
        }

        this.finished = true;
    }
}

package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class NewScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;
    private final Thread extendOutRunnable;
    private final Thread transferRunnable;

    public boolean finished = false;

    public NewScoreMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;

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

        this.robot.intake.closeClaw();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        while (true) {
            if (this.robot.intake.leftClaw.getPosition() == CLOSE_CLAW_POSITION &&
                    this.robot.intake.rightClaw.getPosition() == CLOSE_CLAW_POSITION) {
                break;
            }
        }

        this.transferRunnable.start();

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= 6 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= -6)
                break;
        }

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= 10 &&
                    robot.lift.getVerticalLiftPosition() >= -10)
                break;
        }

        this.robot.intake.transferIntake();
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.robot.intake.openClaw();

        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.robot.intake.groundIntake(0);
//        this.robot.lift.setHorizontalSlide(
//                robot.macroManager.macroList[robot.macroManager.index].state.horizontalPos
//        );
        this.robot.lift.setHorizontalSlide(400);

        this.finished = true;

        this.robot.lift.setVerticalLift(state.verticalPos);

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= state.verticalPos + 10 &&
                    robot.lift.getVerticalLiftPosition() >= state.verticalPos - 10)
                break;
        }

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);



    }
}

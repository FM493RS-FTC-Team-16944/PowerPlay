package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class NewSafeLoadScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;
    private final Thread extendOutRunnable;
    private final Thread transferRunnable;

    public boolean finished = false;

    public NewSafeLoadScoreMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;

        state.horizontalPos = state.horizontalPos + 100;

        this.extendOutRunnable = new Thread(
                new ExtendOutMacro(robot, state)
        );
        this.transferRunnable = new Thread(
                new TransferMacro(robot, state)
        );
    }

    public void run() {
        robot.lift.activateSlideSupport();
        extendOutRunnable.start();

//        if (state.horizontalPos > 1200) {
//            while (true) {
//                if (robot.lift.getHorizontalSlidePosition() > 1000) {
//                    break;
//                }
//            }
//            robot.lift.activateSlideSupport();
//        }

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10) {
                break;
            }
        }

        this.robot.intake.closeClaw();

        try {
            Thread.sleep(100);
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

//        if (state.horizontalPos > 1200) {
//            while (true) {
//                if (robot.lift.getHorizontalSlidePosition() < 1000) {
//                    break;
//                }
//            }
//            robot.lift.deactivateSlideSupport();
//        }

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= 10 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= -10)
                break;
        }

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= 10 &&
                    robot.lift.getVerticalLiftPosition() >= -10)
                break;
        }

//        try {
//            Thread.sleep(200);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//
//        while (true) {
//            if (this.robot.intake.rotatorClaw.getPosition() == ONE_EIGHTY_ROTATOR_POSITION &&
//                    this.robot.intake.armClaw.getPosition() == DROP_ARM_CLAW_POSITION_B) {
//                break;
//            }
//        }

        this.robot.intake.transferIntake();
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        this.robot.intake.openClaw();
//        try {
//            Thread.sleep(100);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        this.robot.intake.groundIntake(state.armPos);
    }
}

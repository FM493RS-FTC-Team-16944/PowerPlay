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

        while (true) {
            if (this.robot.intake.leftClaw.getPosition() == CLOSE_CLAW_POSITION &&
                    this.robot.intake.rightClaw.getPosition() == CLOSE_CLAW_POSITION) {
                break;
            }
        }

        this.transferRunnable.start();

        while (true) {
            if (robot.lift.horizontalSlide.getCurrentPosition() <= 3 &&
                    robot.lift.horizontalSlide.getCurrentPosition() >= -3)
                break;
        }

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= 1 &&
                    robot.lift.getVerticalLiftPosition() >= -1)
                break;
        }

        this.robot.intake.openClaw();
        this.robot.intake.groundIntake(0);
        this.robot.lift.setHorizontalSlide(
                robot.macroManager.macroList[robot.macroManager.index].state.horizontalPos
        );

        this.finished = true;

        this.robot.lift.setVerticalLift(state.verticalPos);

        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= state.verticalPos + 10 &&
                    robot.lift.getVerticalLiftPosition() >= state.verticalPos - 10)
                break;
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);
    }
}

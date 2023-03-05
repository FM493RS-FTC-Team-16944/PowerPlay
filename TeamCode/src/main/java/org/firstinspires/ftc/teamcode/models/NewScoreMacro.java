package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.RANGE_OF_UNSAFE_VERTICAL_LIFT;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.TILT_THRESHOLD;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.operations.AutonomousOP;
import org.firstinspires.ftc.teamcode.operations.AutonomousOPLeft;

public class NewScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;
    private final Thread extendOutRunnable;
    private final Thread transferRunnable;

    public boolean finished = false;

    public  NewScoreMacro(MecanumDrive robot, GrabPosition state) {
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
        this.robot.macroMode = true;
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
        ElapsedTime timer = new ElapsedTime();

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
            else if (timer.seconds() > 1.5) {
                break;
            }
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

        try {
            Thread.sleep(150);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.robot.intake.groundIntake(state.armPos);


//        try {
//            Thread.sleep(250);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

//        this.robot.lift.setHorizontalSlide(
//                robot.macroManager.macroList[robot.macroManager.index].state.horizontalPos
//        );
        this.robot.lift.setVerticalLift(state.verticalPos);

        if (state.horizontalPos > 700)
            this.robot.lift.setHorizontalSlide(state.horizontalPos - 700);

//        if ((state.horizontalPos - 700) > 1200) {
//            while (true) {
//                if (robot.lift.getHorizontalSlidePosition() > 1000) {
//                    break;
//                }
//            }
//            robot.lift.activateSlideSupport();
//        }


        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= state.verticalPos + 6 &&
                    robot.lift.getVerticalLiftPosition() >= state.verticalPos - 6)
                break;
        }

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

        while (true) {
            if (Math.abs(this.robot.imu.getAngularOrientation().secondAngle) > TILT_THRESHOLD ||
                    Math.abs(this.robot.imu.getAngularOrientation().thirdAngle) > TILT_THRESHOLD) {
                this.robot.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);
                AutonomousOP.tipping = true;
                AutonomousOPLeft.tipping = true;
                break;
            } else if (this.robot.lift.getVerticalLiftPosition() < ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - RANGE_OF_UNSAFE_VERTICAL_LIFT) {
                break;
            }
        }

        this.finished = true;
        this.robot.macroMode = false;

    }
}

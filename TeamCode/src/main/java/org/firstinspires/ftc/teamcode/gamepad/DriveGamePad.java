package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class DriveGamePad {
    private final MecanumDrive robot;
    private final Gamepad gamepad;

    private final boolean macroMode = false;


    private boolean previousA = false;
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousBumpRight = false;
    private boolean previousBumpLeft = false;
    private boolean previousPadRight = false;
    private boolean previousPadLeft = false;
    private boolean highPole = false;
    private boolean mediumPole = false;


    private int intakePosition = 19;
    private int retrievalIndex = 1;
    private boolean invertedRetrieval = false;
    private boolean retrievingCone = false;
    private DcMotorEx selectedLift;


    public DriveGamePad(MecanumDrive robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.robot.lift.verticalLiftEncoder.setTargetPosition(0);
    }

    public void updateRobot() {
        this.robot.strafeR(-gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x);

        this.robot.odometry.liftOdometry();
        this.robot.lift.deactivateSlideSupport();

//        if (this.gamepad.right_trigger > 0) {
//            selectedLift.setPower(this.gamepad.right_trigger);
//        }
//
//        if (this.gamepad.left_trigger > 0) {
//            selectedLift.setPower(-this.gamepad.left_trigger);
//        }
//
//        if (this.gamepad.dpad_left) {
//            selectedLift = robot.lift.horizontalSlide;
//        }
//
//        if (this.gamepad.dpad_right) {
//            selectedLift = robot.lift.verticalLiftEncoder;
//        }

        //robot.lift.horizontalSlide.setPower(0);
        //robot.lift.verticalLiftEncoder.setPower(0);

        if (gamepad.a && gamepad.a != previousA) {
            if (this.robot.intake.leftClaw.getPosition() == OPEN_CLAW_POSITION &&
                    this.robot.intake.rightClaw.getPosition() == OPEN_CLAW_POSITION) {
                this.robot.intake.closeClaw();
            } else {
                this.robot.intake.openClaw();
            }
        }
        previousA = gamepad.a;

        if (gamepad.right_bumper & gamepad.right_bumper != previousBumpRight) {
            retrievingCone = false;
            intakePosition++;
            retrievalIndex = 0;
        }
        previousBumpRight = gamepad.right_bumper;

        if (gamepad.left_bumper & gamepad.left_bumper != previousBumpLeft) {
            retrievingCone = false;
            intakePosition--;
            retrievalIndex = 0;
        }
        previousBumpLeft = gamepad.left_bumper;

        if (gamepad.dpad_right & gamepad.dpad_right != previousPadRight) {
            invertedRetrieval = false;
            retrievingCone = true;
            intakePosition = 0;
            retrievalIndex++;
        }
        previousPadRight = gamepad.dpad_right;

        if (gamepad.dpad_left & gamepad.dpad_left != previousPadLeft) {
            invertedRetrieval = true;
            retrievingCone = true;
            intakePosition = 0;
            retrievalIndex++;
        }
        previousPadLeft = gamepad.dpad_left;

        if (intakePosition % 5 == 0 && !retrievingCone) {
            this.robot.intake.groundIntake(0.00);
        } else if (intakePosition % 5 == 1 && !retrievingCone) {
            this.robot.intake.hangingIntake();
        } else if (intakePosition % 5 == 2 && !retrievingCone) {
            this.robot.intake.rotatedHangingIntake();
        } else if (intakePosition % 5 == 3 && !retrievingCone) {
            this.robot.intake.transferIntake();
        } else if (intakePosition % 5 == 4 && !retrievingCone) {
            this.robot.intake.groundIntake(0.07);          //formerly 0.4
        } else if (retrievalIndex % 2 == 0 && retrievingCone) {
            this.robot.intake.retrievalIntake(invertedRetrieval);
        } else if (retrievalIndex % 2 == 1 && retrievingCone) {
            this.robot.intake.retrievalIntakeUP(invertedRetrieval);
        }

        if (gamepad.x && gamepad.x != previousX) {
            robot.resetOdom();
            //robot.lift.resetLifts();
        }
        previousX = gamepad.x;

        this.robot.lift.verticalLiftEncoder.setPower(1);

        if (gamepad.y && gamepad.y != previousY) {
            if (intakePosition % 5 == 2 || intakePosition % 5 == 3) {
                intakePosition = 21;
                this.robot.intake.openClaw();
            }
            if (!highPole) {
                this.robot.lift.setVerticalLift(HIGH_SCORE_VERTICAL_LIFT_POSITION);
                highPole = true;
                mediumPole = false;
            } else {
                this.robot.lift.setVerticalLift(0);
                highPole = false;
                mediumPole = false;
            }
        }
        previousY = gamepad.y;

        if (gamepad.b && gamepad.b != previousB) {
            if (intakePosition % 5 == 2 || intakePosition % 5 == 3) {
                intakePosition = 21;
                this.robot.intake.openClaw();
            }
            if (!mediumPole) {
                this.robot.lift.setVerticalLift(MEDIUM_SCORE_VERTICAL_LIFT_POSITION);
                mediumPole = true;
                highPole = false;
            } else {
                this.robot.lift.setVerticalLift(0);
                highPole = false;
                mediumPole = false;
            }
        }
        previousB = gamepad.b;

        this.robot.lift.setHorizontalSlide(0);


    }


}

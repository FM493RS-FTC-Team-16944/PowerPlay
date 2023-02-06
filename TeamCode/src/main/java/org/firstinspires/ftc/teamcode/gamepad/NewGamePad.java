package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.GrabPosition;
import org.firstinspires.ftc.teamcode.models.NewScoreMacro;

public class NewGamePad {
    private final MecanumDrive robot;
    private final Gamepad gamepad;

    private boolean macroMode = false;
    private DcMotorEx selectedLift;
    private int intakePosition = 0;

    public NewGamePad(MecanumDrive robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void updateRobot() {
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );

        if (this.gamepad.right_trigger > 0) {
            selectedLift.setPower(this.gamepad.right_trigger);
        }

        if (this.gamepad.left_trigger > 0) {
            selectedLift.setPower(-this.gamepad.left_trigger);
        }

        if (this.gamepad.dpad_left) {
            selectedLift = robot.lift.horizontalSlide;
        }

        if (this.gamepad.dpad_right) {
            selectedLift = robot.lift.verticalLiftEncoder;
        }

        robot.lift.horizontalSlide.setPower(0);
        robot.lift.verticalLiftEncoder.setPower(0);

        if (gamepad.a) {
            if (this.robot.intake.leftClaw.getPosition() == OPEN_CLAW_POSITION && this.robot.intake.rightClaw.getPosition() == OPEN_CLAW_POSITION) {
                this.robot.intake.openClaw();
            } else {
                this.robot.intake.closeClaw();
            }

        }

        if (gamepad.b) {
            robot.resetOdom();
        }

        if (gamepad.start) {
            macroMode = !macroMode;
        }

        if (macroMode) {

        }

        if (macroMode) {
            if (gamepad.x) {
                int horizontalTarget = 1000;
                int verticalTarget = ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;

                if(robot.lift.getHorizontalSlidePosition() > 0) {
                    horizontalTarget = robot.lift.getHorizontalSlidePosition();

                    robot.lift.setHorizontalSlide(0);
                }

                if(robot.lift.getVerticalLiftPosition() > 0) {
                    verticalTarget = robot.lift.getVerticalLiftPosition();

                    robot.lift.setVerticalLift(0);
                }

                NewScoreMacro scoringMacro = new NewScoreMacro(
                        robot, new GrabPosition(ARM_CLAW_POSITION_FIFTH_CONE, horizontalTarget, verticalTarget)
                );

                Thread macroThread = new Thread(scoringMacro);
                macroThread.start();
            }
        } else {
            if (gamepad.x) {
                intakePosition++;
                intakePosition %= 4;

                if (intakePosition == 0) {
                    this.robot.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
                } else if (intakePosition == 1) {
                    this.robot.intake.hangingIntake();
                } else if (intakePosition == 2) {
                    this.robot.intake.rotatedHangingIntake();
                } else if (intakePosition == 3) {
                    this.robot.intake.transferIntake();
                }
            }
        }


    }


}

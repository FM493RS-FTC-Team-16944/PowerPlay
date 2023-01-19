package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_NEUTRAL;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LOW_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Config
public class TestGamePad {
    private final Gamepad gamepad;
    private final MecanumDrive robot;
    private final Telemetry telemetry;

    public static int armClawPosition = 1;
    public static int rotatorPosition = 1;
    public static int clawTiltPosition = 1;
    public static int horizontalSlidePosition = 1;
    public static int verticalLiftPosition = 1;

    public TestGamePad(MecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
        this.gamepad = hardwareGamepad;
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void updateRobot() {
        this.robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );

        this.telemetry.addData("updating score", "yes");

        if (gamepad.a) {
            this.robot.openClaw();
        }

        if (gamepad.b) {
            this.robot.closeClaw();
        }

        if (gamepad.x) {
            if (armClawPosition == 1) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_FIRST_CONE);
            } else if (armClawPosition == 2) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_SECOND_CONE);
            } else if (armClawPosition == 3) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_THIRD_CONE);
            } else if (armClawPosition == 4) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_FOURTH_CONE);
            } else if (armClawPosition == 5) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_FIFTH_CONE);
            } else if (armClawPosition == 6) {
                this.robot.setArmClawPosition(ARM_CLAW_POSITION_NEUTRAL);
            }
        }

        if (gamepad.y) {
            if (rotatorPosition == 1) {
                this.robot.setRotatorClawPosition(NORMAL_ROTATOR_POSITION);
            } else if (rotatorPosition == 2) {
                this.robot.setRotatorClawPosition(ONE_EIGHTY_ROTATOR_POSITION);
            }
        }

        if (gamepad.dpad_up) {
            if (clawTiltPosition == 1) {
                this.robot.setTiltClawPosition(NEUTRAL_CLAW_TILT_POSITION);
            } else if (clawTiltPosition == 2) {
                this.robot.setTiltClawPosition(UP_CLAW_TILT_POSITION);
            } else if (clawTiltPosition == 3) {
                this.robot.setTiltClawPosition(DROP_CLAW_TILT_POSITION);
            }
        }

        if (gamepad.dpad_down) {
            if (horizontalSlidePosition == 1) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE);
            } else if (horizontalSlidePosition == 2) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE);
            } else if (horizontalSlidePosition == 3) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE);
            } else if (horizontalSlidePosition == 4) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE);
            } else if (horizontalSlidePosition == 5) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE);
            } else if (horizontalSlidePosition == 6) {
                this.robot.setHorizontalSlide(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
            }
        }

        if (gamepad.dpad_left) {
            this.telemetry.addData("ok", verticalLiftPosition);
            if (verticalLiftPosition == 1) {
                this.robot.setVerticalLift(HIGH_SCORE_VERTICAL_LIFT_POSITION);
            } else if (verticalLiftPosition == 2) {
                this.robot.setVerticalLift(MEDIUM_SCORE_VERTICAL_LIFT_POSITION);
            } else if (verticalLiftPosition == 3) {
                this.robot.setVerticalLift(LOW_SCORE_VERTICAL_LIFT_POSITION);
            } else if (verticalLiftPosition == 4) {
                this.robot.setVerticalLift(NEUTRAL_VERTICAL_LIFT_POSITION);
            }
        }
    }
}

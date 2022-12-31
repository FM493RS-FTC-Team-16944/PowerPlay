package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ARM_CLAW_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ARM_CLAW_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ARM_CLAW_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive.VERTICAL_LIFT_POWER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.models.Mode;

@Config
public class TestGamePad {
    private final Gamepad gamepad;
    private final SampleMecanumDrive robot;
    private final Telemetry telemetry;

    public static int numberCone = 1;

    public TestGamePad(SampleMecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
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

        if(gamepad.a) {
            this.robot.openClaw();
        }

        if(gamepad.b) {
            this.robot.closeClaw();
        }

        if(gamepad.x) {
            this.robot.setArmClawPosition(ARM_CLAW_POSITION_FIRST_CONE);
        }

        if(gamepad.y) {
            this.robot.setRotatorClawPosition(NORMAL_ROTATOR_POSITION);
        }
    }
}

package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.OpenClose;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ManualGamePad {
    private final Gamepad gamepad;
    private final MecanumDrive robot;
    private final Telemetry telemetry;

    public static int armClawPosition = 1;
    public static int rotatorPosition = 1;
    public static int clawTiltPosition = 1;
    public static int horizontalSlidePosition = 1;
    public static int verticalLiftPosition = 1;

    private int position = 3;
    private int lift = 1;
    private int slide = 1;

    OpenClose ClawOpen = OpenClose.CLOSE;
    OpenClose rightClawOpen = OpenClose.CLOSE;


    public Motor currentLift;
    public boolean prevSwitched = false;

    public boolean prevRightClaw = false;
    public boolean prevClaw = false;
    public boolean prevLeftLift = false;
    public boolean prevRightLift = false;
    public boolean prevA = false;
    public boolean prevB = false;
    public boolean prevY = false;
    private boolean prevX = false;

    public ManualGamePad(MecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
        this.gamepad = hardwareGamepad;
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void updateRobot() {

        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x
                )
        );

        Pose2d poseEstimate = robot.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("horizontal slide position", robot.lift.getHorizontalSlidePosition());
        telemetry.addData("vertical slide position", robot.lift.getVerticalLiftPosition());



        if (gamepad.x && gamepad.x != prevX) {
            if (ClawOpen == OpenClose.OPEN) {
                ClawOpen = OpenClose.CLOSE;
                this.robot.intake.closeClaw();
            } else {
                ClawOpen = OpenClose.OPEN;
                this.robot.intake.openClaw();
            }
        }
        prevX = gamepad.x;

        if (gamepad.dpad_up) {
            this.robot.lift.setVerticalLift(this.robot.lift.getVerticalLiftPosition()+30);
        }
        if (gamepad.dpad_down) {
            this.robot.lift.setVerticalLift(this.robot.lift.getVerticalLiftPosition()-30);
        }
        if (gamepad.dpad_right) {
            this.robot.lift.setHorizontalSlide(this.robot.lift.getHorizontalSlidePosition()+30);
        }
        if (gamepad.dpad_left) {
            this.robot.lift.setHorizontalSlide(this.robot.lift.getHorizontalSlidePosition()-30);
        }

        if (gamepad.a && gamepad.a != prevA) {
            lift++;
            lift %= 2;
            if (lift==0) {
                this.robot.lift.setVerticalLift(0);
            } else if (lift == 1) {
                this.robot.lift.setVerticalLift(1200);
            }
        }
        prevA = gamepad.a;

        if (gamepad.y && gamepad.y != prevY) {
            lift++;
            lift %= 2;
            if (lift==0) {
                this.robot.lift.setVerticalLift(0);
            } else if (lift == 1) {
                this.robot.lift.setVerticalLift(1200);
            }
        }
        prevY = gamepad.y;

        if (gamepad.b && gamepad.b != prevB) {
            position++;
            position %= 4;
            if (position==0) {
                this.robot.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            } else if (position == 1) {
                this.robot.intake.hangingIntake();
            } else if (position == 2) {
                this.robot.intake.rotatedHangingIntake();
            } else if (position == 3) {
                this.robot.intake.transferIntake();
            }
        }
        prevB = gamepad.b;

        telemetry.addData("Position:", position);
        telemetry.addData("claw", ClawOpen);



    }
}
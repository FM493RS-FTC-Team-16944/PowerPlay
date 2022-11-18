package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.OpenClose;


public class ButtonGamePad {
    private final Gamepad gamepad;

    Lift leftLiftUp = Lift.DOWN;
    Lift rightLiftUp = Lift.DOWN;

    OpenClose leftClawOpen = OpenClose.CLOSE;
    OpenClose rightClawOpen = OpenClose.CLOSE;

    private final RobotHardware hardware;
    private final RobotMovement movement;

    public boolean prevRightClaw = false;
    public boolean prevLeftClaw = false;
    public boolean prevLeftLift = false;
    public boolean prevRightLift = false;
    public boolean prevA = false;
    public boolean prevB = false;
    public boolean prevY = false;

    public ButtonGamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.gamepad = hardwareGamepad;
    }

    public void updateRobot() {
        if (gamepad.dpad_down && gamepad.y && !prevLeftLift) {
            leftLiftUp = Lift.UP;
            this.hardware.driveTrain.leftLift.goToPosition(3725, 0.3);
        } else if (gamepad.dpad_down && gamepad.b && !prevLeftLift) {
            leftLiftUp = Lift.UP;
            this.hardware.driveTrain.leftLift.goToPosition(2800, 0.3);
        } else if (gamepad.dpad_down && gamepad.a && !prevLeftLift) {
            leftLiftUp = Lift.UP;
            this.hardware.driveTrain.leftLift.goToPosition(1900, 0.3);
        } else if (gamepad.dpad_down && !prevLeftLift) {
            if (leftLiftUp == Lift.UP) {
                leftLiftUp = Lift.DOWN;
                this.hardware.driveTrain.leftLift.goToPosition(0, 0.3);
            } else {
                leftLiftUp = Lift.UP;
                this.hardware.driveTrain.leftLift.goToPosition(3725, 0.3);
            }
        }
        prevLeftLift = gamepad.dpad_down;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevY = gamepad.y;

        if (gamepad.dpad_up && gamepad.y && !prevRightLift) {
            rightLiftUp = Lift.UP;
            this.hardware.driveTrain.rightLift.goToPosition(2900, 0.3);
        } else if (gamepad.dpad_up && gamepad.b && !prevRightLift) {
            rightLiftUp = Lift.UP;
            this.hardware.driveTrain.rightLift.goToPosition(2150, 0.3);
        } else if (gamepad.dpad_up && gamepad.a && !prevRightLift) {
            rightLiftUp = Lift.UP;
            this.hardware.driveTrain.rightLift.goToPosition(1250, 0.3);
        } else if (gamepad.dpad_up && !prevRightLift) {
            if (rightLiftUp == Lift.UP) {
                rightLiftUp = Lift.DOWN;
                this.hardware.driveTrain.rightLift.goToPosition(0, 0.3);
            } else {
                rightLiftUp = Lift.UP;
                this.hardware.driveTrain.rightLift.goToPosition(2900, 0.3);
            }
        }

        prevRightLift = gamepad.dpad_up;
        prevA = gamepad.a;
        prevB = gamepad.b;

        if (gamepad.dpad_left && gamepad.dpad_left != prevLeftClaw) {
            prevLeftClaw = false;
            if (leftClawOpen == OpenClose.OPEN) {
                leftClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.leftClaw.setPosition(0);
            } else {
                leftClawOpen = OpenClose.OPEN;
                hardware.driveTrain.leftClaw.setPosition(0.75);
            }
        }
        prevLeftClaw = gamepad.dpad_left;

        if (gamepad.dpad_right && gamepad.dpad_right != prevRightClaw) {
            prevRightClaw = false;
            if (rightClawOpen == OpenClose.OPEN) {
                rightClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.rightClaw.setPosition(0);
            } else {
                rightClawOpen = OpenClose.OPEN;
                hardware.driveTrain.rightClaw.setPosition(0.75);
            }
        }
        prevRightClaw = gamepad.dpad_right;
    }
}
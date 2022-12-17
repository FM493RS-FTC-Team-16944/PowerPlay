package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.OpenClose;
import org.firstinspires.ftc.teamcode.hardware.Motor;


public class ButtonGamePad {
    private final Gamepad gamepad;

    OpenClose leftClawOpen = OpenClose.CLOSE;
    OpenClose rightClawOpen = OpenClose.CLOSE;

    private final RobotHardware hardware;

    public boolean prevRightClaw = false;
    public boolean prevLeftClaw = false;

    public Motor currentLift;
    public boolean prevSwitched = false;


    public ButtonGamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.gamepad = hardwareGamepad;
        this.currentLift = hardware.driveTrain.leftLift;
    }

    public void updateRobot() {

        if (gamepad.dpad_left && gamepad.dpad_left != prevLeftClaw) {
            prevLeftClaw = false;
            if (leftClawOpen == OpenClose.OPEN) {
                leftClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.leftClaw.setPosition(0.6);
            } else {
                leftClawOpen = OpenClose.OPEN;
                hardware.driveTrain.leftClaw.setPosition(0);
            }
        }
        prevLeftClaw = gamepad.dpad_left;

        if (gamepad.dpad_right && gamepad.dpad_right != prevRightClaw) {
            prevRightClaw = false;
            if (rightClawOpen == OpenClose.OPEN) {
                rightClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.rightClaw.setPosition(0.6);
            } else {
                rightClawOpen = OpenClose.OPEN;
                hardware.driveTrain.rightClaw.setPosition(0);
            }
        }
        prevRightClaw = gamepad.dpad_right;

        if (gamepad.dpad_left && gamepad.dpad_left != prevLeftClaw) {
            prevLeftClaw = false;
            if (leftClawOpen == OpenClose.OPEN) {
                leftClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.leftClaw.setPosition(0.6);
            } else {
                leftClawOpen = OpenClose.OPEN;
                hardware.driveTrain.leftClaw.setPosition(0.75);
            }
        }
        prevLeftClaw = gamepad.dpad_left;

        prevRightClaw = gamepad.dpad_right;

        if(gamepad.right_bumper) {
            currentLift = hardware.driveTrain.rightLift;
        }

        if(gamepad.left_bumper) {
            currentLift = hardware.driveTrain.leftLift;
        }

        prevSwitched = (gamepad.right_bumper || gamepad.left_bumper);

        if(gamepad.right_trigger > 0){
            currentLift.setPower(gamepad.right_trigger);
        }else if(gamepad.left_trigger > 0){
            currentLift.setPower(-gamepad.left_trigger);
        }else if(gamepad.left_trigger == 0 && gamepad.right_trigger == 0){
            currentLift.setPower(0);
        }
    }
}
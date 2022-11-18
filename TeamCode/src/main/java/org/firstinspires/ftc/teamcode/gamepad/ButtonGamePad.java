package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.ServoMotor;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.OpenClose;


public class ButtonGamePad {
    private final Gamepad gamepad;

    private final RobotHardware hardware;
    private final RobotMovement movement;


    public Motor currentLift;
    public ServoMotor currentClaw;
    public int[] currentHeights;
    public boolean prevSwitched = false;

    public boolean prevOpen = false;
    public boolean prevClosed = false;
    public boolean prevUp = false;
    public boolean prevDown = false;
    public boolean prevA = false;
    public boolean prevB = false;
    public boolean prevY = false;

    private int[] leftHeights = {0, 1250, 2150, 2900};
    private int[] rightHeights = {0, 1900, 2800, 3725};

    public ButtonGamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.gamepad = hardwareGamepad;
    }

    public void updateRobot() {
        if (gamepad.dpad_up && gamepad.y && !prevUp) {
            currentLift.goToPosition(currentHeights[3], 0.3);
        } else if (gamepad.dpad_up && gamepad.b && !prevUp) {
            currentLift.goToPosition(currentHeights[2], 0.3);
        } else if (gamepad.dpad_up && gamepad.a && !prevUp) {
            this.hardware.driveTrain.leftLift.goToPosition(currentHeights[1], 0.3);
        }else if(gamepad.dpad_up && !prevUp){
            currentLift.goToPosition(currentHeights[3],0.3);
        }

        if (gamepad.dpad_up && !prevDown) {
            currentLift.goToPosition(0, 0.3);
        }

        prevUp = gamepad.dpad_up;
        prevDown = gamepad.dpad_down;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevY = gamepad.y;


        if (gamepad.dpad_left && !prevClosed) {
            currentClaw.setPosition(0);
        }

        prevClosed = gamepad.dpad_left;

        if (gamepad.dpad_right && !prevOpen) {
            currentClaw.setPosition(0.75);
        }
        prevOpen = gamepad.dpad_right;

        if(!prevSwitched){
            if(gamepad.left_bumper){
                this.currentLift = hardware.driveTrain.leftLift;
                this.currentClaw = hardware.driveTrain.leftClaw;
                this.currentHeights = leftHeights;
            }else if(gamepad.right_bumper){
                this.currentLift = hardware.driveTrain.rightLift;
                this.currentClaw = hardware.driveTrain.rightClaw;
                this.currentHeights = rightHeights;
            }
        }
        prevSwitched = (gamepad.left_bumper || gamepad.right_bumper);
    }


}
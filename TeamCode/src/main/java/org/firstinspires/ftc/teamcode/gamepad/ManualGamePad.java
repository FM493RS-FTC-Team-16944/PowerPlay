package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.ServoMotor;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.OpenClose;

public class ManualGamePad {
    private final Gamepad gamepad;

    Lift leftLiftUp = Lift.DOWN;
    Lift rightLiftUp = Lift.DOWN;

    OpenClose leftClawOpen = OpenClose.CLOSE;
    OpenClose rightClawOpen = OpenClose.CLOSE;

    private final RobotHardware hardware;
    private final RobotMovement movement;

    public Motor currentLift;
    public boolean prevSwitched = false;

    public boolean prevRightClaw = false;
    public boolean prevLeftClaw = false;
    public boolean prevLeftLift = false;
    public boolean prevRightLift = false;
    public boolean prevA = false;
    public boolean prevB = false;
    public boolean prevY = false;

    public ManualGamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.gamepad = hardwareGamepad;
        this.currentLift = hardware.driveTrain.leftLift;
    }

    public void updateRobot() {

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
        prevRightClaw = gamepad.dpad_right;

        if(gamepad.right_trigger > 0 && gamepad.left_trigger > 0 && !prevSwitched){
            if(currentLift == hardware.driveTrain.leftLift){
                currentLift = hardware.driveTrain.rightLift;
            }else{
                currentLift = hardware.driveTrain.leftLift;
            }
        }
        prevSwitched = (gamepad.right_trigger > 0 && gamepad.left_trigger > 0);

        if(gamepad.right_trigger > 0){
            hardware.driveTrain.setEncodersMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentLift.setPower(0.3);
        }else if(gamepad.left_trigger > 0){
            hardware.driveTrain.setEncodersMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentLift.setPower(-0.3);
        }else if(gamepad.left_trigger == 0 && gamepad.right_trigger == 0){
            hardware.driveTrain.setEncodersMode(DcMotor.RunMode.RUN_USING_ENCODER);
            currentLift.setPower(0);
        }

    }
}

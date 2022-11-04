package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;

import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.LiftMacro;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.OpenClose;


public class GamePad {
    private final Gamepad gamepad;
    Lift leftLiftUp = Lift.DOWN;
    Lift rightLiftUp = Lift.DOWN;
    OpenClose leftClawOpen = OpenClose.OPEN;
    OpenClose rightClawOpen = OpenClose.OPEN;
    private final RobotHardware hardware;
    private final RobotMovement movement;


    private int i;

    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;

        this.gamepad = hardwareGamepad;
        this.i = 0;
    }

    public void updateRobot() {
        if (hardware.currentMode == Mode.DRIVER_CONTROL) {
            double x = gamepad.left_stick_x;
            double y = gamepad.left_stick_y; // Remember, this is reversed!
            double h = gamepad.right_stick_x;

            movement.strafeR(x, y, h);
        }

        if (gamepad.a) {
            hardware.resetAngle();
        }
        LiftMacro liftMacroUp0l = new LiftMacro(movement, 0, hardware.driveTrain, "leftLift");
        Thread lt0 = new Thread(liftMacroUp0l);
        LiftMacro liftMacroUp1l = new LiftMacro(movement, 2000, hardware.driveTrain, "leftLift");
        Thread lt1 = new Thread(liftMacroUp1l);
        LiftMacro liftMacroUp2l = new LiftMacro(movement, 4000, hardware.driveTrain, "leftLift");
        Thread lt2 = new Thread(liftMacroUp2l);
        LiftMacro liftMacroUp3l = new LiftMacro(movement, 5000, hardware.driveTrain, "leftLift");
        Thread lt3 = new Thread(liftMacroUp3l);

        LiftMacro liftMacroUp0r = new LiftMacro(movement, 0, hardware.driveTrain, "rightLift");
        Thread rt0 = new Thread(liftMacroUp0r);
        LiftMacro liftMacroUp1r = new LiftMacro(movement, 2000, hardware.driveTrain, "rightLift");
        Thread rt1 = new Thread(liftMacroUp1r);
        LiftMacro liftMacroUp2r = new LiftMacro(movement, 4000, hardware.driveTrain, "rightLift");
        Thread rt2 = new Thread(liftMacroUp2r);
        LiftMacro liftMacroUp3r = new LiftMacro(movement, 5000, hardware.driveTrain, "rightLift");
        Thread rt3 = new Thread(liftMacroUp3r);

        if(gamepad.dpad_down && gamepad.a){
            leftLiftUp = Lift.UP;
            lt1.start();
        }else if(gamepad.dpad_down && gamepad.b){
            leftLiftUp = Lift.UP;
            lt2.start();
        }else if(gamepad.dpad_down){
            if(leftLiftUp == Lift.UP) {
                leftLiftUp = Lift.DOWN;
                lt0.start();
            }else{
                leftLiftUp = Lift.UP;
                lt3.start();
            }
        }

        if(gamepad.dpad_up && gamepad.a){
            rightLiftUp = Lift.UP;
            rt1.start();
        }else if(gamepad.dpad_up && gamepad.b){
            leftLiftUp = Lift.UP;
            rt2.start();
        }else if(gamepad.dpad_up){
            if(rightLiftUp == Lift.UP) {
                rightLiftUp = Lift.DOWN;
                rt0.start();
            }else{
                rightLiftUp = Lift.UP;
                rt3.start();
            }
        }

        if(gamepad.dpad_left){
            if(leftClawOpen == OpenClose.OPEN){
                leftClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.leftClaw.setPosition(0);
            }else{
                leftClawOpen = OpenClose.OPEN;
                hardware.driveTrain.leftClaw.setPosition(1);
            }
        }

        if(gamepad.dpad_right){
            if(rightClawOpen == OpenClose.OPEN){
                rightClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.rightClaw.setPosition(0);
            }else{
                rightClawOpen = OpenClose.OPEN;
                hardware.driveTrain.rightClaw.setPosition(1);
            }
        }
    }
}
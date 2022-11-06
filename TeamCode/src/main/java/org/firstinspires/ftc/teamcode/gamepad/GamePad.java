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
    public boolean prevX = false;



    private int i;

    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        this.hardware = robot.hardware;
        this.movement = robot.movement;
        this.gamepad = hardwareGamepad;
        this.i = 0;
    }

    public void updateRobot() {
        if (hardware.currentMode == Mode.DRIVER_CONTROL) {
            double x = -gamepad.left_stick_x;
            double y = -gamepad.left_stick_y; // Remember, this is reversed!
            double h = gamepad.right_stick_x;

            movement.strafe(x, y, h);
        }

        if (gamepad.a) {
            hardware.resetAngle();
        }
        LiftMacro liftMacroUp0l = new LiftMacro(movement, 0, hardware.driveTrain, "leftLift");
        Thread lt0 = new Thread(liftMacroUp0l);
        LiftMacro liftMacroUp1l = new LiftMacro(movement, 1500, hardware.driveTrain, "leftLift");
        Thread lt1 = new Thread(liftMacroUp1l);
        LiftMacro liftMacroUp2l = new LiftMacro(movement, 2500, hardware.driveTrain, "leftLift");
        Thread lt2 = new Thread(liftMacroUp2l);
        LiftMacro liftMacroUp3l = new LiftMacro(movement, 4000, hardware.driveTrain, "leftLift");
        Thread lt3 = new Thread(liftMacroUp3l);

        LiftMacro liftMacroUp0r = new LiftMacro(movement, 0, hardware.driveTrain, "rightLift");
        Thread rt0 = new Thread(liftMacroUp0r);
        LiftMacro liftMacroUp1r = new LiftMacro(movement, 1500, hardware.driveTrain, "rightLift");
        Thread rt1 = new Thread(liftMacroUp1r);
        LiftMacro liftMacroUp2r = new LiftMacro(movement, 2500, hardware.driveTrain, "rightLift");
        Thread rt2 = new Thread(liftMacroUp2r);
        LiftMacro liftMacroUp3r = new LiftMacro(movement, 4000, hardware.driveTrain, "rightLift");
        Thread rt3 = new Thread(liftMacroUp3r);
        if (gamepad.dpad_down && gamepad.a && !prevLeftLift && !prevA) {
            leftLiftUp = Lift.UP;
            lt1.start();
        } else if (gamepad.dpad_down && gamepad.b &&!prevLeftLift &&!prevB) {
            leftLiftUp = Lift.UP;
            lt2.start();
        } else if(gamepad.dpad_down && gamepad.x &&!prevLeftLift &&!prevX){
            leftLiftUp = Lift.UP;
            lt3.start();
        }else if (gamepad.dpad_down && !prevLeftLift) {
            if (leftLiftUp == Lift.UP) {
                leftLiftUp = Lift.DOWN;
                lt0.start();
            } else {
                leftLiftUp = Lift.UP;
                lt3.start();
            }
        }
        prevLeftLift = gamepad.dpad_down;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevX = gamepad.x;

        if(gamepad.dpad_up && gamepad.a && !prevRightLift && !prevA){
            rightLiftUp = Lift.UP;
            rt1.start();
        }else if(gamepad.dpad_up && gamepad.b && !prevRightLift && !prevB){
            rightLiftUp = Lift.UP;
            rt2.start();
        }else if(gamepad.dpad_up && gamepad.x && !prevRightLift && !prevX){
            rightLiftUp = Lift.UP;
            rt3.start();
        }else if(gamepad.dpad_up && !prevRightLift){
            if(rightLiftUp == Lift.UP) {
                rightLiftUp = Lift.DOWN;
                rt0.start();
            }else{
                rightLiftUp = Lift.UP;
                rt3.start();
            }
        }
        prevRightLift = gamepad.dpad_up;
        prevA = gamepad.a;
        prevB = gamepad.b;
        prevX = gamepad.x;


        if(gamepad.dpad_left && gamepad.dpad_left != prevLeftClaw){
            prevLeftClaw = false;
            if(leftClawOpen == OpenClose.OPEN){
                leftClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.leftClaw.setPosition(0);
            }else{
                leftClawOpen = OpenClose.OPEN;
                hardware.driveTrain.leftClaw.setPosition(1);
            }
        }
        prevLeftClaw = gamepad.dpad_left;

        if(gamepad.dpad_right && gamepad.dpad_right != prevRightClaw){
            prevRightClaw = false;
            if(rightClawOpen == OpenClose.OPEN){
                rightClawOpen = OpenClose.CLOSE;
                hardware.driveTrain.rightClaw.setPosition(0);
            }else{
                rightClawOpen = OpenClose.OPEN;
                hardware.driveTrain.rightClaw.setPosition(1);
            }
        }
        prevRightClaw = gamepad.dpad_right;
    }
}
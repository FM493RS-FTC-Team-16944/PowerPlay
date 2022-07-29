package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.models.Mapping.BUTTON_A;
import static org.firstinspires.ftc.teamcode.models.Mapping.BUTTON_X;
import static org.firstinspires.ftc.teamcode.models.Mapping.BUTTON_Y;
import static org.firstinspires.ftc.teamcode.models.Mapping.DPAD_DOWN;
import static org.firstinspires.ftc.teamcode.models.Mapping.DPAD_UP;
import static org.firstinspires.ftc.teamcode.models.Mapping.LEFT_BUMPER;
import static org.firstinspires.ftc.teamcode.models.Mapping.LEFT_TRIGGER;
import static org.firstinspires.ftc.teamcode.models.Mapping.MOVEMENT;
import static org.firstinspires.ftc.teamcode.models.Mapping.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.models.Mapping.RIGHT_TRIGGER;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.gamepad.easypad.GamePadMap;
import org.firstinspires.ftc.teamcode.gamepad.easypad.IGamePad;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.modes.LiftMacro;

public class GamePad extends IGamePad {
    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        super(robot, hardwareGamepad);
    }

    @GamePadMap(map = MOVEMENT)
    public void onMovement(float left_x, float left_y, float right_x, float right_y) {
        if(hardware.currentMode == Mode.DRIVER_CONTROL) {
            double x = left_x;
            double y = -left_y; // Remember, this is reversed!
            double h = right_x;

            movement.strafeR(x, y, h);
        }
    }

    @GamePadMap(map = BUTTON_A)
    public void onButtonA(boolean a) {
        switch (hardware.currentMode) {
            case DRIVER_CONTROL:
                hardware.currentMode = Mode.AUTOMATIC_CONTROL;

            case AUTOMATIC_CONTROL:
                hardware.currentMode = Mode.DRIVER_CONTROL;
        }
    }

    @GamePadMap(map = BUTTON_X)
    public void onButtonX(boolean x) {
        movement.toggleClaw();
    }

    @GamePadMap(map = BUTTON_Y)
    public void onButtonY(boolean y) {
        movement.toggleArm();
    }

    @GamePadMap(map = LEFT_TRIGGER)
    public void onLeftTrigger(float trigger) {
        movement.activateIntake(trigger);
    }

    @GamePadMap(map = LEFT_BUMPER)
    public void onLeftBumper(boolean leftBumper) {
        movement.activateIntake();
    }

    @GamePadMap(map = RIGHT_TRIGGER)
    public void onRightTrigger(float trigger) {
        movement.activateFlywheel(trigger);
    }

    @GamePadMap(map = RIGHT_BUMPER)
    public void onRightBumper(boolean rightBumper){
        movement.activateFlywheel();
    }

    @GamePadMap(map = DPAD_UP)
    public void onDPadUp(boolean dPadUp) {
        LiftMacro liftMacro = new LiftMacro(movement, Lift.UP);
        Thread t1 = new Thread(liftMacro);
        t1.start();
    }

    @GamePadMap(map = DPAD_DOWN)
    public void onDPadDown(boolean dPadDown) {
        LiftMacro liftMacro = new LiftMacro(movement, Lift.DOWN);
        Thread t1 = new Thread(liftMacro);
        t1.start();
    }
}
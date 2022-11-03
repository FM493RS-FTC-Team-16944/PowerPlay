package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;

public class GamePad {
    private final Gamepad gamepad;

    private final RobotHardware hardware;
    private final RobotMovement movement;

    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousIn = false;
    private boolean previousFly = false;
    private boolean previousUp = false;
    private boolean previousDown = false;
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

            movement.strafe(x, y, h);
        }
    }
        // change mode
//        if (gamepad.a) {
//            this.hardware.resetAngle();
            /*
            switch (hardware.currentMode) {
                case DRIVER_CONTROL:
                    hardware.currentMode = Mode.AUTOMATIC_CONTROL;

                case AUTOMATIC_CONTROL:
                    hardware.currentMode = Mode.DRIVER_CONTROL;
            }

             */
        //}



    //}
}
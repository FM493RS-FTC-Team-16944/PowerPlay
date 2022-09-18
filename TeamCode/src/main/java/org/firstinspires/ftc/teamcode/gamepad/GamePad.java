package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.gamepad.easypad.IGamePad;

public class GamePad extends IGamePad {
    public GamePad(Robot robot, Gamepad hardwareGamepad) {
        super(robot, hardwareGamepad);
    }
}
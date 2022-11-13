package org.firstinspires.ftc.teamcode.operations;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.GamePad;
import org.firstinspires.ftc.teamcode.gamepad.easypad.IGamePad;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.models.XyhVector;

import java.util.LinkedHashMap;

@TeleOp(name = "TeleOp")
public class TeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public GamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;
        gamepad = new GamePad(robot,gamepad1);

        hardware.driveTrain.resetDriveEncoders();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();
            hardware.odometry.updateOdometryReadings();

            gamepad.updateRobot();
        }
    }


}
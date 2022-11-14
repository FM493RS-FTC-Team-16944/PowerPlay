package org.firstinspires.ftc.teamcode.operations;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.ButtonGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "SplitTeleOp")
public class SplitTeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public MovementGamePad gamepad;
    private ButtonGamePad buttonGamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;

        gamepad = new MovementGamePad(robot,gamepad1);
        buttonGamepad = new ButtonGamePad(robot, gamepad2);

        hardware.driveTrain.resetDriveEncoders();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();

            gamepad.updateRobot();
            buttonGamepad.updateRobot();

            hardware.outputReadings();
        }
    }


}
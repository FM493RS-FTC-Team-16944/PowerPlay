package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.gamepad.CombinedGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;

@TeleOp(name = "ManualTeleOp")
public class ManualTeleOp extends LinearOpMode {

    public Robot robot;

    public RobotHardware hardware;
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;

        gamepad = new ManualGamePad(robot,gamepad1);

        hardware.driveTrain.resetDriveEncoders();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();

            gamepad.updateRobot();

            hardware.outputReadings();
        }
    }
}

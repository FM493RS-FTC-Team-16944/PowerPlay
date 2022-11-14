package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.gamepad.CombinedGamePad;

@TeleOp(name = "UnifiedTeleOp")
public class UnifiedTeleOp extends LinearOpMode {

    public Robot robot;

    public RobotHardware hardware;
    public CombinedGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        robot = new Robot(this);
        hardware = robot.hardware;

        gamepad = new CombinedGamePad(robot,gamepad1);

        hardware.driveTrain.resetDriveEncoders();

        while (opModeIsActive() && !isStopRequested()) {
            hardware.odometry.update();

            gamepad.updateRobot();

            hardware.outputReadings();
        }
    }
}

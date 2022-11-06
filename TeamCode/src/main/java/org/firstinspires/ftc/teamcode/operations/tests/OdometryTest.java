package org.firstinspires.ftc.teamcode.operations.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.gamepad.GamePad;
import org.firstinspires.ftc.teamcode.gamepad.easypad.IGamePad;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name="Odometry Test")
public class OdometryTest extends LinearOpMode{
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
//            hardware.odometry.update();
            hardware.odometry.updateIMUHead();
            hardware.odometry.updateOdometryReadings();
            gamepad.updateRobot();
        }
    }
}

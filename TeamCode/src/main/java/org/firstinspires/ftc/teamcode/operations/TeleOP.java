package org.firstinspires.ftc.teamcode.operations;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GamePad;
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
            gamepad.updateRobot();
            hardware.odometry.updateIMUHead();


            /**ODOM DEBUG **/
            // telemetry.addData("RightEncoder", hardware.currentRightPos);
            // telemetry.addData("LeftEncoder", hardware.currentLeftPos);
            // telemetry.addData("AuxEncoder", hardware.currentAuxPos);
            // telemetry.addData("IMU Angle", hardware.globalAngle);
            // telemetry.addData("RawLeft", hardware.leftEncoder.getCurrentPosition());
            // telemetry.addData("RawRight", hardware.rightEncoder.getCurrentPosition());
            // telemetry.addData("RawHori", hardware.auxEncoder.getCurrentPosition());


            telemetry.update();
        }
    }


}
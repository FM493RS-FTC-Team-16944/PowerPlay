package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.DriveGamePad;
import org.firstinspires.ftc.teamcode.gamepad.RandomGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.Random;

@TeleOp(name = "TestTeleOP")
public class TestTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        RandomGamePad gamePad = new RandomGamePad(drive, this.gamepad1);

        drive.setPoseEstimate(PoseStorage.currentPos);

        drive.odometry.liftOdometry();

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();

            telemetry.update();

        }
    }
}

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
            this.telemetry.addData("Horizontal Limit Switched Pressed: ", drive.lift.horizontalLimitSwitch.isPressed());
            this.telemetry.addData("Vertical Limit Switched Pressed: ", drive.lift.verticalLimitSwitch.isPressed());
            this.telemetry.addData("vertical lift position", drive.lift.getVerticalLiftPosition());
            this.telemetry.addData("horizontal slide position", drive.lift.getHorizontalSlidePosition());
            this.telemetry.addData(" x axis", drive.imu.getAngularOrientation().firstAngle);
            this.telemetry.addData(" y axis", drive.imu.getAngularOrientation().secondAngle);
            this.telemetry.addData(" z axis", drive.imu.getAngularOrientation().thirdAngle);

            gamePad.updateRobot();

            telemetry.update();
        }
    }
}

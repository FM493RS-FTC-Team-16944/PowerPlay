package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.NewGamePad;
import org.firstinspires.ftc.teamcode.gamepad.driverGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        driverGamePad gamePad = new driverGamePad(drive, this.gamepad1);

        drive.setPoseEstimate(PoseStorage.currentPos);

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();

            this.telemetry.addData("angle", drive.odometry.getHeading());
            telemetry.update();

            drive.updateLifts();
        }
    }
}

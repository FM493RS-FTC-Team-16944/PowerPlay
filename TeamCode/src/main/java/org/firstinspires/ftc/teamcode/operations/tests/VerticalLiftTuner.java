package org.firstinspires.ftc.teamcode.operations.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.TestGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Config
@TeleOp(group = "drive")
public class VerticalLiftTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        TestGamePad gamePad = new TestGamePad(drive, this.gamepad1, this.telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@TeleOp(name = "ManualTeleOp")
public class ManualTeleOp extends LinearOpMode {
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        ManualGamePad gamePad = new ManualGamePad(drive, this.gamepad1, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@TeleOp(name = "MacroTeleOp")
public class MacroTeleOp extends LinearOpMode {
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        MacroGamePad gamePad = new MacroGamePad(drive, this.gamepad1, this.telemetry);

        drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            telemetry.update();
        }
    }
}

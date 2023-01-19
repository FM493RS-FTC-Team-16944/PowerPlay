package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@TeleOp(name = "MovementTeleOp")
public class MovementTeleOp extends LinearOpMode {

    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MovementGamePad gamePad = new MovementGamePad(drive, this.gamepad1, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            drive.odometry.update();
            drive.outputOdomReadings(telemetry);
            telemetry.update();
        }
    }
}

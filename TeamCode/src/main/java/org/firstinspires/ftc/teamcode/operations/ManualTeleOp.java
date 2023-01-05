package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.gamepad.CombinedGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.TestGamePad;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

@TeleOp(name = "ManualTeleOp")
public class ManualTeleOp extends LinearOpMode {

    public Robot robot;

    public RobotHardware hardware;
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ManualGamePad gamePad = new ManualGamePad(drive, this.gamepad1, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            telemetry.update();
        }
    }
}

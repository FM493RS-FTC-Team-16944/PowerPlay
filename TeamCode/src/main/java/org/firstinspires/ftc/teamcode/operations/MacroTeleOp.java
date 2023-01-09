package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.gamepad.CombinedGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.TestGamePad;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

@TeleOp(name = "MacroTeleOp")
public class MacroTeleOp extends LinearOpMode {

    public Robot robot;

    public RobotHardware hardware;
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MacroGamePad gamePad = new MacroGamePad(drive, this.gamepad1, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            telemetry.update();
        }
    }
}

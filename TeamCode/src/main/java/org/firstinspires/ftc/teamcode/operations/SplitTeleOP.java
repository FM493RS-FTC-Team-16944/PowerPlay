package org.firstinspires.ftc.teamcode.operations;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.ButtonGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.models.ScoringMacro;

@TeleOp(name = "SplitTeleOp")
public class SplitTeleOP extends LinearOpMode {
    public Robot robot;

    public RobotHardware hardware;
    public MovementGamePad gamepad;
    private ButtonGamePad buttonGamepad;
    public static ScoringMacro currentThread;

    @Override
    public void runOpMode() {
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MovementGamePad moveGamePad = new MovementGamePad(drive,this.gamepad1, this.telemetry);
        MacroGamePad macroGamePad = new MacroGamePad(drive, this.gamepad2, this.telemetry);

        while (opModeIsActive() && !isStopRequested()) {
            drive.odometry.update();
            moveGamePad.updateRobot();
            macroGamePad.updateRobot();
            drive.outputOdomReadings(telemetry);

            telemetry.update();
        }
        if(isStopRequested()){
            currentThread.complete = true;
        }
    }


}
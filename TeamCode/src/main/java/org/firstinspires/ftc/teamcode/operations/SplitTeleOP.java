package org.firstinspires.ftc.teamcode.operations;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.ScoringMacro;

@TeleOp(name = "SplitTeleOp")
public class SplitTeleOP extends LinearOpMode {

    public MovementGamePad gamepad;
    public static ScoringMacro currentThread;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        MovementGamePad moveGamePad = new MovementGamePad(drive,this.gamepad1, this.telemetry);
        MacroGamePad macroGamePad = new MacroGamePad(drive, this.gamepad2, this.telemetry);


        while (opModeIsActive() && !isStopRequested()) {
            drive.odometry.update();

            moveGamePad.updateRobot();
            macroGamePad.updateRobot();

            drive.outputOdomReadings(telemetry);
        }
        if(isStopRequested()){
//            currentThread.complete = true;
//            currentThread.horizontalPID.complete = true;
//            currentThread.zeroHorizPID.complete = true;
//            currentThread.poleHeightPID.complete = true;
//            currentThread.zeroHeightPID.complete = true;
        }
    }


}
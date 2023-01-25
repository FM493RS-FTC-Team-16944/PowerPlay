package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "NewAutoCycleTest")
public class NewAutoCycle extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);

        while (opModeIsActive()) {
            if (!drive.macroManager.isFinished()) {
                drive.macroManager.startScoring();
            }
        }
    }
}


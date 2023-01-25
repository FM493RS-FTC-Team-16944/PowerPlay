package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.CYCLE_GRAB_POSITIONS;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.commands.HorizontalSlideCommand;
import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.GrabPosition;
import org.firstinspires.ftc.teamcode.models.NewScoreMacro;

import java.time.Instant;

@Autonomous(name = "NewAutoCycleTest")
public class NewAutoCycle extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);

        GrabPosition firstCone = CYCLE_GRAB_POSITIONS[0];
        GrabPosition secondCone = CYCLE_GRAB_POSITIONS[1];
        GrabPosition thirdCone = CYCLE_GRAB_POSITIONS[2];
        GrabPosition fourthCone = CYCLE_GRAB_POSITIONS[3];
        GrabPosition fifthCone = CYCLE_GRAB_POSITIONS[4];

        NewScoreMacro firstConeMacro = new NewScoreMacro(drive, firstCone, telemetry);
        NewScoreMacro secondConeMacro = new NewScoreMacro(drive, secondCone, telemetry);
        NewScoreMacro thirdConeMacro = new NewScoreMacro(drive, thirdCone, telemetry);
        NewScoreMacro fourthConeMacro = new NewScoreMacro(drive, fourthCone, telemetry);
        NewScoreMacro fifthConeMacro = new NewScoreMacro(drive, fifthCone, telemetry);

        Thread firstThread = new Thread(firstConeMacro);
        Thread secondThread = new Thread(secondConeMacro);
        Thread thirdThread = new Thread(thirdConeMacro);
        Thread fourthThread = new Thread(fourthConeMacro);
        Thread fifthThread = new Thread(fifthConeMacro);

        firstThread.start();
        while(opModeIsActive()) {
            if(firstConeMacro.finished) {
                secondThread.start();
                while(opModeIsActive()) {
                    if(secondConeMacro.finished) {
                        thirdThread.start();
                        while(opModeIsActive()) {
                            if(thirdConeMacro.finished) {
                                fourthThread.start();
                                while(opModeIsActive()) {
                                    if(fourthConeMacro.finished) {
                                        fifthThread.start();
                                        while(opModeIsActive()) {
                                            if(fifthConeMacro.finished) {
                                                stop();
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }


    }
}


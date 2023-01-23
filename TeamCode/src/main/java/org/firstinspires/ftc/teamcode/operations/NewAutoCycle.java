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

        GrabPosition state = CYCLE_GRAB_POSITIONS[0];

        NewScoreMacro macro = new NewScoreMacro(drive, state, telemetry);
        Thread thread = new Thread(macro);
        thread.start();

        while(opModeIsActive()) {
            if(macro.finished) {
                stop();
            }
        }
    }
}


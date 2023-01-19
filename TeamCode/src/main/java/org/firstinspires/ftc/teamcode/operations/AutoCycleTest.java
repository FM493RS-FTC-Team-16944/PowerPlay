package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.CYCLE_GRAB_POSITIONS;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous(name = "AutoCycleTest")
public class AutoCycleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new AutoCycleCommand(drive, CYCLE_GRAB_POSITIONS[0])
                )
        );
    }
}


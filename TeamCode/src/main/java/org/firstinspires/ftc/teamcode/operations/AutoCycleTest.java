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

import java.time.Instant;

@Autonomous(name = "AutoCycleTest")
public class AutoCycleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);

        GrabPosition state = CYCLE_GRAB_POSITIONS[0];

        CommandScheduler.getInstance().registerSubsystem(drive.intake);
        CommandScheduler.getInstance().registerSubsystem(drive.lift);

        CommandScheduler.getInstance().onCommandExecute((Command command) -> {
            this.telemetry.addData("just ran: ", command.getName());
            this.telemetry.update();
        });

        WaitUntilCommand command = new WaitUntilCommand(() -> drive.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 &&
                drive.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10);
        InstantCommand closeClawCommand = new InstantCommand(drive.intake::closeClaw);
        HorizontalSlideCommand horizontalSlideCommand = new HorizontalSlideCommand(drive.lift, state.horizontalPos);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> drive.intake.groundIntake(state.armPos)),
                                new InstantCommand(drive.intake::openClaw),
                                new InstantCommand(() -> drive.lift.setHorizontalSlide(state.horizontalPos))
                        ),
                        new WaitUntilCommand(() -> drive.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 &&
                                drive.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10),
                        new InstantCommand(drive.intake::closeClaw, drive.intake),
                        new ParallelCommandGroup(
                                new InstantCommand(drive.intake::transferIntake),
                                new InstantCommand(() -> drive.lift.setHorizontalSlide(ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION))
                        )
                )
        );

        while (opModeIsActive()) {
            this.telemetry.addData("horizontal slide", drive.lift.horizontalSlide.getCurrentPosition());
            this.telemetry.addData("is command done", horizontalSlideCommand.isFinished());
            this.telemetry.addData("is close claw command done", closeClawCommand.isFinished());

            this.telemetry.update();
        }
    }
}


package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.CYCLE_GRAB_POSITIONS;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.AutoCycleCommand;
import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.GrabPosition;

@Autonomous(name = "AutoCycleTest")
public class AutoCycleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap);

        GrabPosition state = CYCLE_GRAB_POSITIONS[0];

        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new InstantCommand(() -> drive.intake.groundIntake(state.armPos)),
                        new InstantCommand(drive.intake::openClaw),
                        new InstantCommand(() -> drive.lift.setHorizontalSlide(state.horizontalPos))
                ),
                new WaitUntilCommand(() -> {
                    this.telemetry.addData("horizontal slide in command", drive.lift.horizontalSlide.getCurrentPosition());
                    this.telemetry.update();
                    return drive.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 &&
                            drive.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10;
                }).andThen(new InstantCommand(drive.intake::closeClaw)),
                new WaitCommand(75)
                        .andThen(
                                new ParallelCommandGroup(
                                        new InstantCommand(drive.intake::transferIntake),
                                        new InstantCommand(() -> drive.lift.setHorizontalSlide(ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION))
                                )
                        )
        );

        while (opModeIsActive()) {
            this.telemetry.addData("horizontal slide", drive.lift.horizontalSlide.getCurrentPosition());
            // this.telemetry.addData("is command done", command.isFinished());

            this.telemetry.update();
        }
    }
}


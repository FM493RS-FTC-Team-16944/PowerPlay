package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.GrabPosition;

public class AutoCycleCommand extends SequentialCommandGroup {
    public AutoCycleCommand(MecanumDrive robot, GrabPosition state) {
        super(
                // in parallel
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.intake.groundIntake(state.armPos)),
                                new InstantCommand(robot.intake::openClaw),
                                new InstantCommand(() -> robot.lift.setHorizontalSlide(state.horizontalPos))
                        ),
                        new WaitUntilCommand(() ->
                                robot.lift.horizontalSlide.getCurrentPosition() <= state.horizontalPos + 10 ||
                                        robot.lift.horizontalSlide.getCurrentPosition() >= state.horizontalPos - 10
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(robot.intake::closeClaw),
                                new WaitCommand(75)
                                        .andThen(
                                                new ParallelCommandGroup(
                                                        new InstantCommand(robot.intake::transferIntake),
                                                        new InstantCommand(() -> robot.lift.setHorizontalSlide(ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION))
                                                )
                                        )
                        ),
                        new WaitUntilCommand(() -> robot.lift.horizontalSlide.getCurrentPosition() >= ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION),
                        new InstantCommand(robot.intake::openClaw),
                        new InstantCommand(() -> robot.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION)),
                        new WaitUntilCommand(() ->
                                robot.lift.getVerticalLiftPosition() <= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION + 10 ||
                                        robot.lift.getVerticalLiftPosition() >= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - 10
                        ),
                        new InstantCommand(() -> robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION))
                )
        );
    }
}
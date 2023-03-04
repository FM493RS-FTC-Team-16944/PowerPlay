package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequence;

public class NewSafeScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;

    private final Thread newSafeLoadMacro;
    private final Thread newSafeRaiseMacro;

    private final TrajectorySequence polePosition;
    private final TrajectorySequence loadPosition;

    public boolean finished = false;

    public NewSafeScoreMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;

        this.newSafeLoadMacro = new Thread(
                new NewSafeLoadScoreMacro(
                        robot, state
                )
        );

        this.newSafeRaiseMacro = new Thread(
                new NewSafeRaiseLiftScoreMacro(
                        robot, state
                )
        );

        this.polePosition = this.robot.trajectorySequenceBuilder(new Pose2d(31, -14.25, Math.toRadians(180)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(7, -23, Math.toRadians(224)))
                .build();

        this.loadPosition = this.robot.trajectorySequenceBuilder(this.polePosition.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(31, -14.25, Math.toRadians(180)))
                .build();
    }

    public void run() {
        this.robot.macroMode = true;

        this.newSafeLoadMacro.start();

        while(true) {
            if(!this.newSafeLoadMacro.isAlive()) {
                break;
            }
        }

        this.robot.followTrajectorySequence(this.polePosition);

        this.newSafeRaiseMacro.start();

        while(true) {
            if(!this.newSafeRaiseMacro.isAlive()) {
                break;
            }
        }

        this.robot.followTrajectorySequence(this.loadPosition);

        this.finished = true;
        this.robot.macroMode = false;
    }
}

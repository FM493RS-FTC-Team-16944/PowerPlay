package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.VerticalLiftPID;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private MecanumDrive drive;

    AprilTagDetection detector = new AprilTagDetection();

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        VerticalLiftPID zeroHeightPID;

        Pose2d startPose = new Pose2d(40, -65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        drive.outputOdomReadings(telemetry);


        Trajectory offWall = drive.trajectoryBuilder(startPose, true)
                .lineToLinearHeading((new Pose2d(34, -60, Math.toRadians(180))),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .build();
        Trajectory strafeToPole = drive.trajectoryBuilder(offWall.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .strafeLeft(55)
                .build();



        waitForStart();


        drive.followTrajectory(offWall);
        drive.turn(Math.toRadians(-90));
        drive.followTrajectory(strafeToPole);
        drive.turn(Math.toRadians(-20));





            //drive.followTrajectory(parkingSpots[lastDestination]);
    }



}
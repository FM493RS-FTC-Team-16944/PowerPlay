package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private XyhVector lastDestination = new XyhVector(-14, 102, 0); // prev -181 4 0
    private SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        Trajectory forward = drive.trajectoryBuilder(startPose)
                .strafeRight(10)
                .forward(5)
                .build();

        Trajectory lastDestination;


        waitForStart();

        while (opModeIsActive()) {

            sleep(1000);

            AprilTagDetection detections = drive.detector.detectObjects();

            switch (detections.id) {
                case 0:
                case 3:
                case 6:
            }

            lastDestination = drive.trajectoryBuilder(forward.end())
                    .splineToLinearHeading(new Pose2d(this.lastDestination.x, this.lastDestination.y, Math.toRadians(this.lastDestination.h)), Math.toRadians(0))
                    .build();

            sleep(4000);

            drive.followTrajectory(forward);

            drive.followTrajectory(lastDestination);
        }
    }
}
package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;


@Autonomous
@Config
public class RoadRunnerTest extends LinearOpMode {
    private MecanumDrive drive;



    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

//        Trajectory forward = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-2.5,30.8,Math.toRadians(170)))
//                .build();
        Trajectory forwardMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-30,30))
                .build();
        drive.setPoseEstimate(startPose);
        waitForStart();


//
//
        //sleep(4000);
        //set hanging intake for better driving

        drive.followTrajectory(forwardMiddle);

    }



}
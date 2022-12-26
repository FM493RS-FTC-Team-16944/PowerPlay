package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectory.sequence.TrajectorySegment;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous
@Config
public class NewAutonomous extends LinearOpMode {
    public SampleMecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new SampleMecanumDrive(this.hardwareMap);

        Trajectory forwardTrajectory = this.robot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 50, -93))
                .build();

        waitForStart();

        if(!isStopRequested()) return;

        this.robot.followTrajectoryAsync(forwardTrajectory);
    }
}

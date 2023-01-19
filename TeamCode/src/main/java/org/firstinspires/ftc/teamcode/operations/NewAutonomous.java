package org.firstinspires.ftc.teamcode.operations;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Autonomous
@Config
public class NewAutonomous extends LinearOpMode {
    public MecanumDrive robot;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new MecanumDrive(this.hardwareMap);

        Trajectory forwardTrajectory = this.robot.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(0, 50, -93))
                .build();

        waitForStart();

        if(!isStopRequested()) return;

        this.robot.followTrajectoryAsync(forwardTrajectory);
    }
}

package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, this.telemetry);

        Pose2d startPose = new Pose2d(40, -65, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        drive.outputOdomReadings(telemetry);

        TrajectorySequence cyclePosition = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading((new Vector2d(48.5, -55)),
                        MecanumDrive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH),
                        MecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .turn(Math.toRadians(-90))
                .strafeLeft(51)
                .turn(Math.toRadians(-14))
                .forward(2)
                .build();

        TrajectorySequence parkingSpot2 = drive.trajectorySequenceBuilder(cyclePosition.end())
                .lineToConstantHeading(new Vector2d(34,-12))
                .build();

        TrajectorySequence parkingSpot1 = drive.trajectorySequenceBuilder(parkingSpot2.end())
                .lineToConstantHeading(new Vector2d(10,-12))
                .build();

        TrajectorySequence parkingSpot3 = drive.trajectorySequenceBuilder(parkingSpot2.end())
                .lineToConstantHeading(new Vector2d(57,-12))
                .build();

        TrajectorySequence[] parkingSpots = {parkingSpot1, parkingSpot2, parkingSpot3};
        waitForStart();

        AprilTagDetection detection = drive.detector.detectObjects();
            int destinationIndex = 0;

        if(detection != null) {
            switch (detection.id) {
                case 0:
                    destinationIndex = 0;
                case 3:
                    destinationIndex = 1;
                case 6:
                    destinationIndex = 2;
                default:
                    destinationIndex = 0;
            }
        }else{
            destinationIndex = 0;
        }
        this.telemetry.addData("Path", destinationIndex);
        this.telemetry.update();

        drive.followTrajectorySequence(cyclePosition);

        drive.lift.setVerticalLift(ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION);
        drive.intake.groundIntake(0);

        while (true) {
            if (drive.lift.getVerticalLiftPosition() <= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION + 10 &&
                    drive.lift.getVerticalLiftPosition() >= ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION - 10)
                break;
        }

        drive.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

//        while (opModeIsActive()) {
//            if (!drive.macroManager.isFinished()) {
//                drive.macroManager.startScoring();
//            }
//        }

        drive.followTrajectorySequence(parkingSpot2);

        if (destinationIndex != 1) {
            drive.followTrajectorySequence(parkingSpots[destinationIndex]);
        }
    }
}
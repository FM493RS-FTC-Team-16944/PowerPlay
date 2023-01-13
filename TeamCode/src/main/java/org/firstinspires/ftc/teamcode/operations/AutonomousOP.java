package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_NEUTRAL_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.VERTICAL_LIFT_POWER;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.models.VerticalLiftPID;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.openftc.apriltag.AprilTagDetection;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;


@Autonomous
@Config
public class AutonomousOP extends LinearOpMode {
    private SampleMecanumDrive drive;


    AprilTagDetection detector = new AprilTagDetection();

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);

        VerticalLiftPID zeroHeightPID;

        Pose2d startPose = new Pose2d(38, -62, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

//        Trajectory forward = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(-2.5,30.8,Math.toRadians(170)))
//                .build();
        Trajectory forwardMiddle = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(34,-32))
                .build();
        Trajectory forward = drive.trajectoryBuilder(forwardMiddle.end())
                .lineToConstantHeading(new Vector2d(34,10))
                .build();
        Trajectory back = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(34,-15,Math.toRadians(160)))
                .build();
//        Trajectory turn = drive.trajectoryBuilder(back.end())
//                .lineToSplineHeading(new Pose2d(34.5,-3, Math.toRadians(170)))
//                .build();
        Trajectory byCone = drive.trajectoryBuilder(forward.end().plus(new Pose2d(0,0,Math.toRadians(-100))))
                .lineToConstantHeading(new Vector2d(32.5, -3))
                .build();


        Trajectory middleDestination = drive.trajectoryBuilder(forward.end())
                .lineToSplineHeading(new Pose2d(-32.25,-12,Math.toRadians(90)))
                .build();

        Trajectory leftDestination = drive.trajectoryBuilder(middleDestination.end())
                .strafeLeft(20)
                .build();

        Trajectory rightDestination = drive.trajectoryBuilder(middleDestination.end())
                .strafeRight(20)
                .build();

        Trajectory[] parkingSpots = new Trajectory[]{leftDestination,middleDestination,rightDestination};
        int lastDestination = 1;


        waitForStart();


        drive.hangingIntake();
            drive.closeClaw();

//            sleep(1000);
////
//            AprilTagDetection detections = drive.detector.detectObjects();
//            if(detections != null) {
//                switch (detections.id) {
//                    case 0:
//                        lastDestination = 0;
//                    case 3:
//                        lastDestination = 1;
//                    case 6:
//                        lastDestination = 2;
//                }
//                telemetry.addData("Detected Object", detections.id);
//            }else{
//                telemetry.addLine("No object found");
//            }
//            telemetry.update();



//
//
           //sleep(4000);
            //set hanging intake for better driving

            drive.followTrajectory(forwardMiddle);
            drive.followTrajectory(forward);
            drive.followTrajectory(back);
//            drive.turn(Math.toRadians(-100));
//            drive.followTrajectory(byCone);
//
//            //place preloaded cone
//            this.telemetry.addLine("Launching Cone 1");
//            drive.setVerticalLift(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            sleep(3000);
//            drive.setVerticalLift(NEUTRAL_VERTICAL_LIFT_POSITION);
//            sleep(3000);

//            //cycle loaded cones
//
//            //1st Cone
//            drive.groundIntake(ARM_CLAW_POSITION_FIRST_CONE);
//            drive.openClaw();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE);
//            drive.closeClaw();
//            drive.hangingIntake();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
//            drive.rotatedHangingIntake();
//            drive.openClaw();
//            drive.verticalLiftEncoder.setTargetPosition(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            drive.verticalLiftEncoder.setTargetPosition(NEUTRAL_VERTICAL_LIFT_POSITION);
//            //2nd Cone
//            drive.groundIntake(ARM_CLAW_POSITION_SECOND_CONE);
//            drive.openClaw();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE);
//            drive.closeClaw();
//            drive.hangingIntake();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
//            drive.rotatedHangingIntake();
//            drive.openClaw();
//            drive.verticalLiftEncoder.setTargetPosition(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            drive.verticalLiftEncoder.setTargetPosition(NEUTRAL_VERTICAL_LIFT_POSITION);
//            //3rd Cone
//            drive.groundIntake(ARM_CLAW_POSITION_THIRD_CONE);
//            drive.openClaw();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE);
//            drive.closeClaw();
//            drive.hangingIntake();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
//            drive.rotatedHangingIntake();
//            drive.openClaw();
//            drive.verticalLiftEncoder.setTargetPosition(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            drive.verticalLiftEncoder.setTargetPosition(NEUTRAL_VERTICAL_LIFT_POSITION);
//            //4th Cone
//            drive.groundIntake(ARM_CLAW_POSITION_FOURTH_CONE);
//            drive.openClaw();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE);
//            drive.closeClaw();
//            drive.hangingIntake();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
//            drive.rotatedHangingIntake();
//            drive.openClaw();
//            drive.verticalLiftEncoder.setTargetPosition(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            drive.verticalLiftEncoder.setTargetPosition(NEUTRAL_VERTICAL_LIFT_POSITION);
//            //5th Cone
//            drive.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
//            drive.openClaw();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE);
//            drive.closeClaw();
//            drive.hangingIntake();
//            drive.horizontalSlide.setTargetPosition(HORIZONTAL_SLIDE_NEUTRAL_POSITION);
//            drive.rotatedHangingIntake();
//            drive.openClaw();
//            drive.verticalLiftEncoder.setTargetPosition(HIGH_SCORE_VERTICAL_LIFT_POSITION);
//            drive.verticalLiftEncoder.setTargetPosition(NEUTRAL_VERTICAL_LIFT_POSITION);

            //drive to parking spot

//            drive.followTrajectory(middleDestination);
//            if(lastDestination != 1) {
//                drive.followTrajectory(parkingSpots[lastDestination]);
//            }
            //drive.followTrajectory(parkingSpots[lastDestination]);
    }



}
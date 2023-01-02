package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double HORIZONTAL_SLIDE_POWER = 0.00;
    public static double VERTICAL_LIFT_POWER = 0.80;

    public static double OPEN_CLAW_POSITION = 0.80;
    public static double CLOSE_CLAW_POSITION = 1.00;

    public static double ARM_CLAW_POSITION_FIRST_CONE = 0.35;
    public static double ARM_CLAW_POSITION_SECOND_CONE = 0.31;
    public static double ARM_CLAW_POSITION_THIRD_CONE = 0.22;
    public static double ARM_CLAW_POSITION_FOURTH_CONE = 0.20;
    public static double ARM_CLAW_POSITION_FIFTH_CONE = 0.00;

    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIRST_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_SECOND_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_THIRD_CONE = 0.00;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FOURTH_CONE = 0.20;
    public static double SECOND_PART_UP_ARM_CLAW_POSITION_FIFTH_CONE = 0.00;

    public static double DROP_ARM_CLAW_POSITION = 0.48;

    public static double NEUTRAL_CLAW_TILT_POSITION = 0.55;
    public static double UP_CLAW_TILT_POSITION = 0.00;
    public static double DROP_CLAW_TILT_POSITION = 0.00; // this is the tilt when it's gonna drop it on the red thing

    public static double NORMAL_ROTATOR_POSITION = 1.00;
    public static double ONE_EIGHTY_ROTATOR_POSITION = 0.25;

    public static int NEUTRAL_VERTICAL_LIFT_POSITION = 0;
    public static int HIGH_SCORE_VERTICAL_LIFT_POSITION = 1300;
    public static int MEDIUM_SCORE_VERTICAL_LIFT_POSITION = 660;
    public static int LOW_SCORE_VERTICAL_LIFT_POSITION = 290;

    public final AprilTagDetector detector;

    public StandardTrackingWheelLocalizer odometry;
    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;
    public RobotMovement movement;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront,
            verticalLiftNoEncoder, verticalLiftEncoder, horizontalSlide;

    private Servo leftClaw, rightClaw, rotatorClaw, tiltClaw, armClaw;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLiftNoEncoder = hardwareMap.get(DcMotorEx.class, "verticalLiftNoEncoder");
        verticalLiftEncoder = hardwareMap.get(DcMotorEx.class, "verticalLiftEncoder");

        verticalLiftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalLiftNoEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalLiftNoEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        verticalLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalLiftNoEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");

        rotatorClaw = hardwareMap.get(Servo.class, "rotatorClaw");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        tiltClaw = hardwareMap.get(Servo.class, "tiltClaw");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        this.armClaw.setDirection(Servo.Direction.REVERSE);
        this.tiltClaw.setDirection(Servo.Direction.REVERSE);

        this.leftClaw.setDirection(Servo.Direction.REVERSE);

        this.movement = new RobotMovement(this);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }


        leftRear.setDirection(DcMotor.Direction.REVERSE);

        setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));
        odometry = new StandardTrackingWheelLocalizer(hardwareMap);

        detector = new AprilTagDetector(hardwareMap);
        this.detector.initDetector();

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setVerticalLift(int position, double power) {
        this.verticalLiftEncoder.setTargetPosition(position);
        this.verticalLiftNoEncoder.setTargetPosition(position);
        this.verticalLiftEncoder.setPower(power);
        this.verticalLiftNoEncoder.setPower(power);

        this.verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.verticalLiftNoEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setHorizontalSlide(int position) {
        this.horizontalSlide.setTargetPosition(position);
        this.horizontalSlide.setPower(HORIZONTAL_SLIDE_POWER);
    }

    public int getHorizontalSlidePosition() {
        return this.horizontalSlide.getCurrentPosition();
    }

    public int getVerticalLiftPosition() {
        return this.verticalLiftEncoder.getCurrentPosition();
    }

    public void openClaw() {
        this.leftClaw.setPosition(OPEN_CLAW_POSITION);
        this.rightClaw.setPosition(OPEN_CLAW_POSITION);
    }

    public void closeClaw() {
        this.leftClaw.setPosition(CLOSE_CLAW_POSITION);
        this.rightClaw.setPosition(CLOSE_CLAW_POSITION);
    }

    public void setArmClawPosition(double position) {
        this.armClaw.setPosition(position);
    }

    public void setTiltClawPosition(double position) {
        this.tiltClaw.setPosition(position);
    }

    public void setRotatorClawPosition(double position) {
        this.rotatorClaw.setPosition(position);
    }


    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }


    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().zRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}

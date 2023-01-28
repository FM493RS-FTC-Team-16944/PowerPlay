package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.VERTICAL_LIFT_POWER;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.models.GrabPosition;
import org.firstinspires.ftc.teamcode.models.MacroManager;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectory.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetector;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive {

    private final boolean isAuto = false;
    public final IntakeSubsystem intake;
    public final LiftSubsystem lift;
    public final MacroManager macroManager;

    private HardwareMap hardwareMap;

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(4, 0, 0);

    public final AprilTagDetector detector;

    public StandardTrackingWheelLocalizer odometry;
    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double SPEED_CAP_TELEOP = 0.55;
    public static double OMEGA_WEIGHT = 1;
    public RobotMovement movement;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static final GrabPosition[] CYCLE_GRAB_POSITIONS = {
            new GrabPosition(ARM_CLAW_POSITION_FIRST_CONE, HORIZONTAL_SLIDE_AUTON_POSITION_FIRST_CONE),
            new GrabPosition(ARM_CLAW_POSITION_SECOND_CONE, HORIZONTAL_SLIDE_AUTON_POSITION_SECOND_CONE),
            new GrabPosition(ARM_CLAW_POSITION_THIRD_CONE, HORIZONTAL_SLIDE_AUTON_POSITION_THIRD_CONE),
            new GrabPosition(ARM_CLAW_POSITION_FOURTH_CONE, HORIZONTAL_SLIDE_AUTON_POSITION_FOURTH_CONE),
            new GrabPosition(ARM_CLAW_POSITION_FIFTH_CONE, HORIZONTAL_SLIDE_AUTON_POSITION_FIFTH_CONE),
    };

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);
        this.hardwareMap = hardwareMap;
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, new PIDCoefficients(-TRANSLATIONAL_PID.kP, TRANSLATIONAL_PID.kI, TRANSLATIONAL_PID.kD), HEADING_PID,
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
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        this.movement = new RobotMovement(this);

        intake = new IntakeSubsystem(hardwareMap);
        lift = new LiftSubsystem(hardwareMap, telemetry);

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

        odometry = new StandardTrackingWheelLocalizer(hardwareMap);
        setLocalizer(odometry);

        macroManager = new MacroManager(this);

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

    public void resetOdom() {
        this.setLocalizer(new StandardTrackingWheelLocalizer(this.hardwareMap));
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

    public void strafeR(double x, double y, double h) {
        Pose2d currentPosition = odometry.getPoseEstimate();
        XyhVector position = new XyhVector(currentPosition.getX(), currentPosition.getY(), currentPosition.getHeading());
        double xR = x * Math.cos(position.h) - y * Math.sin(position.h);
        double yR = x * Math.sin(position.h) + y * Math.cos(position.h);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);

        double frontLeftPower = (yR - xR + h) / denominator;
        double backLeftPower = (yR + xR + h) / denominator;
        double frontRightPower = (yR + xR - h) / denominator;
        double backRightPower = (yR - xR - h) / denominator;

        leftFront.setPower(SPEED_CAP_TELEOP * frontLeftPower);
        leftRear.setPower(SPEED_CAP_TELEOP * backLeftPower);
        rightFront.setPower(SPEED_CAP_TELEOP * frontRightPower);
        rightRear.setPower(SPEED_CAP_TELEOP * backRightPower);

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

    public void outputOdomReadings(Telemetry telemetry) {
        Pose2d positions = this.getPoseEstimate();

        telemetry.addData("X: ", positions.getX());
        telemetry.addData("Y: ", positions.getY());
        telemetry.addData("Heading: ", positions.getHeading());

        telemetry.update();
    }
}

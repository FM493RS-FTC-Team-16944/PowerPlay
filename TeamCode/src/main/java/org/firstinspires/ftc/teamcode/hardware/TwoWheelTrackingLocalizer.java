package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.AUX_ODOM_LOWERED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.AUX_ODOM_RETRACTED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LEFT_ODOM_LOWERED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LEFT_ODOM_RETRACTED;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;


@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double X_MULTIPLIER = 0.978340365;
    public static double Y_MULTIPLIER = 0.980506457;

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 2 / 2.54; // X is the up and down direction
    public static double PARALLEL_Y = 17 / 2.54; // Y is the strafe direction

    public static double PERPENDICULAR_X = -9.7 / 2.54;
    public static double PERPENDICULAR_Y = 5.6 / 2.54;
    private final Servo leftRetraction;
    private final Servo auxRetraction;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    public Encoder parallelEncoder, perpendicularEncoder;

    private final MecanumDrive drive;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, MecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));

        leftRetraction = hardwareMap.get(Servo.class, "leftRetraction");
        auxRetraction = hardwareMap.get(Servo.class, "auxRetraction");

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public void liftOdometry() {
        this.auxRetraction.setPosition(AUX_ODOM_RETRACTED);
        this.leftRetraction.setPosition(LEFT_ODOM_RETRACTED);
    }

    public void lowerOdometry() {
        this.auxRetraction.setPosition(AUX_ODOM_LOWERED);
        this.leftRetraction.setPosition(LEFT_ODOM_LOWERED);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}

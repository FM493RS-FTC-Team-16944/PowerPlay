package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstants.TRACK_WIDTH;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.DriveConstants;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double X_MULTIPLIER = 0.98; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.016; // Multiplier in the Y direction
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 7.75; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 2.88; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;


    public static double CENTER_WHEEL_OFFSET = 5.60;
    public static double R = 1;
    public static double N = 8192;
    public static double in_per_tick = 2.0 * Math.PI * R / N;

    public XyhVector pos;
    public Pose2d pose2d;

    private double prevLeftEncoder = 0;
    private double prevRightEncoder = 0;
    private double prevHorizontalEncoder = 0;

    private double currentLeftEncoder = 0;
    private double currentRightEncoder = 0;
    private double currentHorizontalEncoder = 0;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backRight"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontRight"));
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

        this.pos = new XyhVector();
        this.pose2d = new Pose2d();

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }

    public List<Integer> getEncoderValues(){
        List<Integer> returnList = new ArrayList<>();
        returnList.add(leftEncoder.getCurrentPosition());
        returnList.add(rightEncoder.getCurrentPosition());
        returnList.add(frontEncoder.getCurrentPosition());
        return returnList;
    }



}

package org.firstinspires.ftc.teamcode.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;

public class Odometry {
    public static final double TRACK_WIDTH = 0.00;
    public static final double CENTER_WHEEL_OFFSET = 0.00;

    private Pose2d position;
    private Rotation2d previousAngle;

    private double prevLeftEncoder;
    private double prevRightEncoder;
    private double prevHorizontalEncoder;

    public Odometry() {
        this.position = new Pose2d();
    }

    public Pose2d getPosition() {
        return this.position;
    }

    public void update() {
        double deltaLeftEncoder = leftEncoderPos - prevLeftEncoder;
        double deltaRightEncoder = rightEncoderPos - prevRightEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;
        
        Rotation2d angle = previousAngle.plus(
            new Rotation2d(
                    (deltaLeftEncoder - deltaRightEncoder) / TRACK_WIDTH
            )
        );
        
        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;
        
        double dw = (angle.minus(previousAngle).getRadians());
        double dx =(deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (CENTER_WHEEL_OFFSET * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);
        Pose2d newPose = this.position.exp(twist2d);
        
        previousAngle = angle;
        
        this.position = new Pose2d(newPose.getTranslation(), angle);
    }
}

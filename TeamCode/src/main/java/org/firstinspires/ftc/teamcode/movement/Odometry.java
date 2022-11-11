package org.firstinspires.ftc.teamcode.movement;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.TelemLog;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.models.MiscMethods;


@Config
public class Odometry {
    public static final double TRACK_WIDTH = 38.10; //make sure to cahnge these two values
    public static final double CENTER_WHEEL_OFFSET = 5.60;

    private final RobotHardware hardware;
    private final TelemLog telemetry;

    public Pose2d position;
    public Rotation2d previousAngle = new Pose2d(0,0, new Rotation2d()).getRotation();



    private double prevLeftEncoder = 0;
    private double prevRightEncoder = 0;
    private double prevHorizontalEncoder = 0;

    public Odometry(RobotHardware robotHardware) {
        this.hardware = robotHardware;
        this.telemetry = robotHardware.telemetry;
        this.position = new Pose2d();
    }

    public Pose2d getPosition() {
        return this.position;
    }

    public void update() {
        MecanumDriveTrain driveTrain = this.hardware.driveTrain;

        double leftEncoderPos = driveTrain.leftEncoder.getCurrentPosition();
        double rightEncoderPos = driveTrain.rightEncoder.getCurrentPosition();
        double horizontalEncoderPos = driveTrain.auxEncoder.getCurrentPosition();

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

        double dx = (deltaLeftEncoder + deltaRightEncoder) / 2;
        double dy = deltaHorizontalEncoder - (CENTER_WHEEL_OFFSET * dw);


        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = this.position.exp(twist2d);
        
        Translation2d newTranslation = newPose.getTranslation().rotateBy(angle);
        Rotation2d newAngle = previousAngle.plus(angle);
        this.position = new Pose2d(newTranslation, newAngle);
        previousAngle = angle;
    }


    public void updateIMUHead(){
        Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        float deltaAngle = angles.firstAngle - hardware.lastAngles.firstAngle;
        hardware.globalAngleI = MiscMethods.angleWrap(deltaAngle);
    }
    public void updateOdometryReadings() {
        this.telemetry.addData("Odometry X Position Centimeters : " , (double)(this.position.getX()));
        this.telemetry.addData("Odometry Y Position Centimeters : " , (double)(this.position.getY()));
        this.telemetry.addData("Odometry H Rotation Degrees : " , (double)(position.getRotation().getDegrees()));
        this.telemetry.addData("IMU Heading Degrees: " , (double)(Math.toDegrees(hardware.globalAngleI)));
//        this.telemetry.addData("Left Lift Encoder: " , (double)(hardware.driveTrain.leftLift.getCurrentPosition()));
//        this.telemetry.addData("Right Lift Encoder: " , (double)(hardware.driveTrain.rightLift.getCurrentPosition()));
        telemetry.update();
    }


}

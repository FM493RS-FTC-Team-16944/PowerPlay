package org.firstinspires.ftc.teamcode.movement;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.util.TelemLog;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import org.firstinspires.ftc.teamcode.models.MiscMethods;




public class Odometry {


    public static final double TRACK_WIDTH = 0.00;
    public static final double CENTER_WHEEL_OFFSET = 0.00;

    private final RobotHardware hardware;
    private final TelemLog telemetry;

    private Pose2d position;
    private Rotation2d previousAngle;



    private double prevLeftEncoder;
    private double prevRightEncoder;
    private double prevHorizontalEncoder;

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
        double dy =(deltaLeftEncoder + deltaRightEncoder) / 2;
        double dx = deltaHorizontalEncoder - (CENTER_WHEEL_OFFSET * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);
        Pose2d newPose = this.position.exp(twist2d);
        
        previousAngle = angle;
        
        this.position = new Pose2d(newPose.getTranslation(), angle);
    }


    public void updateIMUHead(){
        Orientation angles = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        float deltaAngle = angles.firstAngle - hardware.lastAngles.firstAngle;
        hardware.globalAngleI = MiscMethods.angleWrap(deltaAngle);
    }
    public void updateOdometryReadings() {
        this.telemetry.addData("Odometry X Position Centimeters : " , this.position.getTranslation().getX());
        this.telemetry.addData("Odometry Y Position Centimeters : " , this.position.getTranslation().getY());
        this.telemetry.addData("Odometry H Position Centimeters : " , this.position.getHeading());
        this.telemetry.addData("Odometry H Rotation Radians : " , this.position.getRotation().getRadians());
        telemetry.update();
    }


}

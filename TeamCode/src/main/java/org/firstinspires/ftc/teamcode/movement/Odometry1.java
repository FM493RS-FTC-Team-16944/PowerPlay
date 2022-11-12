package org.firstinspires.ftc.teamcode.movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.models.XyhVector;
import org.firstinspires.ftc.teamcode.util.TelemLog;


@Config
public class Odometry1 {
    public static double TRACK_WIDTH = 39.40; //make sure to cahnge these two values
    public static double CENTER_WHEEL_OFFSET = 5.60;
    public static double R = 2.54;
    public static double N = 8192;
    public static double cm_per_tick = 2.0 * Math.PI * R / N;

    private final RobotHardware hardware;
    public final TelemLog telemetry;

    public XyhVector pos = new XyhVector();
    public Pose2d pose2d = new Pose2d();

    private double prevLeftEncoder = 0;
    private double prevRightEncoder = 0;
    private double prevHorizontalEncoder = 0;

    private double currentLeftEncoder = 0;
    private double currentRightEncoder = 0;
    private double currentHorizontalEncoder = 0;

    public Odometry1(RobotHardware robotHardware) {
        this.hardware = robotHardware;
        this.telemetry = robotHardware.telemetry;
        this.pos = new XyhVector();
    }

    public XyhVector getPosition() {
        return this.pos;
    }

    public void update() {

        prevLeftEncoder = currentLeftEncoder;
        prevRightEncoder = currentRightEncoder;
        prevHorizontalEncoder = currentHorizontalEncoder;

        currentRightEncoder = hardware.driveTrain.rightEncoder.getCurrentPosition();
        currentLeftEncoder = hardware.driveTrain.leftEncoder.getCurrentPosition();
        currentHorizontalEncoder = hardware.driveTrain.auxEncoder.getCurrentPosition();

        double dn1 = currentLeftEncoder - prevLeftEncoder;
        double dn2 = currentRightEncoder - prevRightEncoder;
        double dn3 = currentHorizontalEncoder - prevHorizontalEncoder;

        double dtheta = cm_per_tick * (dn2 - dn1) / TRACK_WIDTH;
        double dx = cm_per_tick * (dn1 + dn2) / 2.0;
        double dy = cm_per_tick * (dn3 - (dn2 - dn1) * CENTER_WHEEL_OFFSET / TRACK_WIDTH);

        double theta = pos.h + (dtheta / 2.0);
        pos.y += dx * Math.cos(theta) - dy * Math.sin(theta);
        pos.x += dx * Math.sin(theta) + dy * Math.cos(theta);
        pos.h += dtheta;

        pos.h = pos.h % (2 * Math.PI);

        pose2d = new Pose2d(
                pos.x, pos.y,
                new Rotation2d(pos.h)
        );
    }


    public void updateOdometryReadings() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        this.telemetry.addData("Odometry X Position Centimeters : ", pos.x);
        this.telemetry.addData("Odometry Y Position Centimeters : ", pos.y);
        this.telemetry.addData("Odometry H Rotation Degrees : ", Math.toDegrees(pos.h));
        this.telemetry.addData("IMU Heading Degrees: ", (double) (Math.toDegrees(hardware.globalAngleI)));

        dashboardTelemetry.addData("Odometry X Position Centimeters : ", pos.x);
        dashboardTelemetry.addData("Odometry Y Position Centimeters : ", pos.y);
        dashboardTelemetry.addData("Odometry H Rotation Degrees : ", Math.toDegrees(pos.h));
        dashboardTelemetry.addData("IMU Heading Degrees: ", (double) (Math.toDegrees(hardware.globalAngleI)));
//        this.telemetry.addData("Left Lift Encoder: " , (double)(hardware.driveTrain.leftLift.getCurrentPosition()));
//        this.telemetry.addData("Right Lift Encoder: " , (double)(hardware.driveTrain.rightLift.getCurrentPosition()));

        telemetry.update();
        dashboardTelemetry.update();
    }


}

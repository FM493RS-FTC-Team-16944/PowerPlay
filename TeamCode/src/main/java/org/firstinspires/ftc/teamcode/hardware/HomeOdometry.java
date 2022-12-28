//package org.firstinspires.ftc.teamcode.hardware;
//import com.acmerobotics.dashboard.config.Config;
//import com.arcrobotics.ftclib.geometry.Pose2d;
//import com.arcrobotics.ftclib.geometry.Rotation2d;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.RobotHardware;
//import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
//import org.firstinspires.ftc.teamcode.util.TelemLog;
//
//@Config
//public class HomeOdometry {
//
//    public static double TRACK_WIDTH = 39.40; //make sure to cahnge these two values
//    public static double CENTER_WHEEL_OFFSET = 5.60;
//    public static double R = 1;
//    public static double N = 8192;
//    public static double in_per_tick = 2.0 * Math.PI * R / N;
//
//    private final RobotHardware hardware;
//    public final TelemLog telemetry;
//
//    public XyhVector pos;
//    public Pose2d pose2d;
//
//    private double prevLeftEncoder = 0;
//    private double prevRightEncoder = 0;
//    private double prevHorizontalEncoder = 0;
//
//    private double currentLeftEncoder = 0;
//    private double currentRightEncoder = 0;
//    private double currentHorizontalEncoder = 0;
//
//    public HomeOdometry(HardwareMap robotHardware) {
//        this.hardware = robotHardware;
//        this.telemetry = robotHardware.telemetry;
//
//        this.pos = new XyhVector();
//        this.pose2d = new Pose2d();
//    }
//
//    public XyhVector getPosition() {
//        return this.pos;
//    }
//
//    public void update() {
//        prevLeftEncoder = currentLeftEncoder;
//        prevRightEncoder = currentRightEncoder;
//        prevHorizontalEncoder = currentHorizontalEncoder;
//
//        currentRightEncoder = hardware.driveTrain.rightEncoder.getCurrentPosition();
//        currentLeftEncoder = hardware.driveTrain.leftEncoder.getCurrentPosition();
//        currentHorizontalEncoder = hardware.driveTrain.auxEncoder.getCurrentPosition();
//
//        double dn1 = currentLeftEncoder - prevLeftEncoder;
//        double dn2 = currentRightEncoder - prevRightEncoder;
//        double dn3 = currentHorizontalEncoder - prevHorizontalEncoder;
//
//        double dtheta = in_per_tick * (dn2 - dn1) / TRACK_WIDTH;
//        double dx = in_per_tick * (dn1 + dn2) / 2.0;
//        double dy = in_per_tick * (dn3 - (dn2 - dn1) * CENTER_WHEEL_OFFSET / TRACK_WIDTH);
//        double theta = pos.h + (dtheta / 2.0);
//
//        pos.x += -(dx * Math.cos(theta) - dy * Math.sin(theta));
//        pos.y += dx * Math.sin(theta) + dy * Math.cos(theta);
//        pos.h += AngleUnit.normalizeRadians(dtheta);
//
//        pose2d = new Pose2d(
//                pos.x, pos.y,
//                new Rotation2d(pos.h)
//        );
//    }
//
//    public void updateOdometryReadings() {
//        this.telemetry.addData("Odometry X Position (cm)", RobotHardware.DECIMAL_FORMAT.format(pos.x));
//        this.telemetry.addData("Odometry Y Position (cm)", RobotHardware.DECIMAL_FORMAT.format(pos.y));
//        this.telemetry.addData("Odometry H Rotation (deg)",
//                RobotHardware.DECIMAL_FORMAT.format(Math.toDegrees(pos.h)));
//
////        this.telemetry.addData("Left Lift Encoder: " , (double)(hardware.driveTrain.leftLift.getCurrentPosition()));
////        this.telemetry.addData("Right Lift Encoder: " , (double)(hardware.driveTrain.rightLift.getCurrentPosition()));
//    }
//
//}

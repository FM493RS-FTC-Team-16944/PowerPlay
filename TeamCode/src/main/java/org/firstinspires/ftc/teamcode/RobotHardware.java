package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.util.geometry.XyhVector;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.TelemLog;
import org.firstinspires.ftc.teamcode.vision.ObjectDetector;

import java.math.RoundingMode;
import java.text.DecimalFormat;

public class RobotHardware {
    public static final DecimalFormat DECIMAL_FORMAT;

    //public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();

    static {
        DECIMAL_FORMAT = new DecimalFormat("#.###");

        DECIMAL_FORMAT.setRoundingMode(RoundingMode.FLOOR);
    }

    public MecanumDriveTrain driveTrain;
    public final Robot.State state;
    public Odometry odometry;
    public final TelemLog telemetry;
    public Mode currentMode = Mode.DRIVER_CONTROL;
    public final ObjectDetector detector;

    public HardwareMap hardwareMap;

    public RobotHardware(Robot robot) {
        this.hardwareMap = robot.teleOp.hardwareMap;
        this.state = robot.state;

        this.telemetry = robot.telemetry;


        this.driveTrain = new MecanumDriveTrain(
                "frontLeft",
                "backLeft",
                "frontRight",
                "backRight",
                "leftLift",
                "rightLift",
                "leftEncoder",
                "rightEncoder",
                "backRight",
                "leftClaw",
                "rightClaw",
                hardwareMap,
                this.telemetry
        );

        this.odometry = new Odometry(this);


        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = this.hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
         */

        resetAngle();

        this.detector = new ObjectDetector(this);
    }

    public void outputReadings() {
        this.odometry.updateOdometryReadings();
        this.driveTrain.updateEncoderReadings();

        this.telemetry.update();
    }

    public void resetAngle() {
        // lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        this.odometry.pose2d = new Pose2d(odometry.pose2d.getX(),odometry.pose2d.getX(),new Rotation2d());
        this.odometry.pos = new XyhVector(odometry.pos.x, odometry.pos.y,0);

        // this.odometry.previousAngle = new Rotation2d();
    }
}
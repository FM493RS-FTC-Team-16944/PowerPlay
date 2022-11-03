package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.movement.Odometry;
import org.firstinspires.ftc.teamcode.util.TelemLog;

import java.math.RoundingMode;
import java.text.DecimalFormat;

public class RobotHardware {
    public static final DecimalFormat DECIMAL_FORMAT;

    static {
        DECIMAL_FORMAT = new DecimalFormat("#.###");

        DECIMAL_FORMAT.setRoundingMode(RoundingMode.FLOOR);
    }

    public MecanumDriveTrain driveTrain;
    public final Robot.State state;
    public Odometry odometry;
    public final TelemLog telemetry;
    public Mode currentMode = Mode.DRIVER_CONTROL;

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
                "leftLift",
                hardwareMap,
                this.telemetry
        );

        this.odometry = new Odometry(this);


    }

    public void outputReadings() {
        this.odometry.updateOdometryReadings();
        this.driveTrain.updateEncoderReadings();

        this.telemetry.update();
    }

}
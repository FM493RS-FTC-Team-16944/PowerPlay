package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotMovement {
    RobotHardware hardware;

    public static double SPEED_CAP_AUTON = 0.5;
    public static double SPEED_CAP_TELEOP = 0.5;
    
    RobotMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void strafe(double x, double y, double h) {
        x *= 0.75;
        y *= 0.75;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (y - x + h) / denominator;
        double backLeftPower = (y + x + h) / denominator;
        double frontRightPower = (y + x - h) / denominator;
        double backRightPower = (y - x - h) / denominator;

        hardware.driveTrain.topLeft.setPower(SPEED_CAP_AUTON * frontLeftPower);
        hardware.driveTrain.backLeft.setPower(SPEED_CAP_AUTON * backLeftPower);
        hardware.driveTrain.topRight.setPower(SPEED_CAP_AUTON * frontRightPower);
        hardware.driveTrain.backRight.setPower(SPEED_CAP_AUTON * backRightPower);
    }

    public void strafeR(double x, double y, double h) {
        double xR = x * Math.cos(hardware.odometry.pos.h) - y * Math.sin(hardware.odometry.pos.h);
        double yR = x * Math.sin(hardware.odometry.pos.h) + y * Math.cos(hardware.odometry.pos.h);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);

        double frontLeftPower = (yR - xR + h) / denominator;
        double backLeftPower = (yR + xR + h) / denominator;
        double frontRightPower = (yR + xR - h) / denominator;
        double backRightPower = (yR - xR - h) / denominator;

        hardware.driveTrain.topLeft.setPower(SPEED_CAP_TELEOP * frontLeftPower);
        hardware.driveTrain.backLeft.setPower(SPEED_CAP_TELEOP * backLeftPower);
        hardware.driveTrain.topRight.setPower(SPEED_CAP_TELEOP * frontRightPower);
        hardware.driveTrain.backRight.setPower(SPEED_CAP_TELEOP * backRightPower);

    }
}

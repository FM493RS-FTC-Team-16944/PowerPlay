package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@Config
public class RobotMovement {
    MecanumDrive robot;

    public static double SPEED_CAP_AUTON = 0.5;
    public double SPEED_CAP_TELEOP = 0.6;
    
    public RobotMovement(MecanumDrive robot) {
        this.robot = robot;
    }

    public void strafe(double x, double y, double h) {
        x *= 0.75;
        y *= 0.75;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (y - x + h) / denominator;
        double backLeftPower = (y + x + h) / denominator;
        double frontRightPower = (y + x - h) / denominator;
        double backRightPower = (y - x - h) / denominator;

        robot.setMotorPowers(
                SPEED_CAP_AUTON * frontLeftPower,
                SPEED_CAP_AUTON * backLeftPower,
                SPEED_CAP_AUTON * frontRightPower,
                SPEED_CAP_AUTON * backRightPower
        );
    }

    public void strafeR(double x, double y, double h) {
        Double heading = robot.odometry.getWheelPositions().get(2);

        double xR = x * Math.cos(heading) - y * Math.sin(heading);
        double yR = x * Math.sin(heading) + y * Math.cos(heading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);

        double frontLeftPower = (yR - xR + h) / denominator;
        double backLeftPower = (yR + xR + h) / denominator;
        double frontRightPower = (yR + xR - h) / denominator;
        double backRightPower = (yR - xR - h) / denominator;

        robot.setMotorPowers(
                SPEED_CAP_TELEOP * frontLeftPower,
                SPEED_CAP_TELEOP * backLeftPower,
                SPEED_CAP_TELEOP * frontRightPower,
                SPEED_CAP_TELEOP * backRightPower
        );
    }
}

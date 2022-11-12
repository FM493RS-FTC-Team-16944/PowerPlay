package org.firstinspires.ftc.teamcode;


public class RobotMovement {
    RobotHardware hardware;

    public int open = 0;

    RobotMovement(Robot robot) {
        this.hardware = robot.hardware;
    }

    public void     strafe(double x, double y, double h) {
        x *= 0.75;
        y *= 0.75;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(h), 1);
        double frontLeftPower = (y - x + h) / denominator;
        double backLeftPower = (y + x + h) / denominator;
        double frontRightPower = (y + x - h) / denominator;
        double backRightPower = (y - x - h) / denominator;

        hardware.driveTrain.topLeft.setPower(0.5*frontLeftPower);
        hardware.driveTrain.backLeft.setPower(0.5*backLeftPower);
        hardware.driveTrain.topRight.setPower(0.5*frontRightPower);
        hardware.driveTrain.backRight.setPower(0.5*backRightPower);

    }

    public void strafeR(double x, double y, double h) {
        double xR = x * Math.cos(hardware.globalAngleI) - y * Math.sin(hardware.globalAngleI);
        double yR = x * Math.sin(hardware.globalAngleI) + y * Math.cos(hardware.globalAngleI);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(hardware.globalAngleI), 1);

        double frontLeftPower = (yR - xR + h) / denominator;
        double backLeftPower = (yR + xR + h) / denominator;
        double frontRightPower = (yR + xR - h) / denominator;
        double backRightPower = (yR - xR - h) / denominator;

        hardware.driveTrain.topLeft.setPower(0.5*frontLeftPower);
        hardware.driveTrain.backLeft.setPower(0.5*backLeftPower);
        hardware.driveTrain.topRight.setPower(0.5*frontRightPower);
        hardware.driveTrain.backRight.setPower(0.5*backRightPower);
    }


}

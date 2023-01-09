package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.OpenClose;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MovementGamePad {
    private final Gamepad gamepad;
    private final SampleMecanumDrive robot;
    private final Telemetry telemetry;

    public Boolean prevDpadUp = false;
    public Boolean prevDpadDown = false;
    public Boolean prevX = false;


    public MovementGamePad(SampleMecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
        this.gamepad = hardwareGamepad;
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void updateRobot() {

        robot.strafeR(
                -gamepad.left_stick_x,
                -gamepad.left_stick_y,
                gamepad.right_stick_x
        );

        if(gamepad.dpad_up && !prevDpadUp){
            robot.resetVerticalSlidePosition();
        }

        prevDpadUp = gamepad.dpad_up;

        if(gamepad.dpad_down && !prevDpadDown){
            robot.resetHorizontalSlidePosition();
        }

        prevDpadDown = gamepad.dpad_down;

        if(gamepad.x && !prevX){
            robot.resetOdom();
        }




    }
}
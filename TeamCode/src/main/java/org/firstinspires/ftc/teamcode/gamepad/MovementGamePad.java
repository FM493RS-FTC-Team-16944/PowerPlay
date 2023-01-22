package org.firstinspires.ftc.teamcode.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MovementGamePad {
    private final Gamepad gamepad;
    private final MecanumDrive robot;
    private final Telemetry telemetry;

    public boolean prevDpadUp = false;
    public boolean prevDpadDown = false;
    public boolean prevLeftBump = false;
    public boolean prevRightBump = false;
    public boolean prevX = false;
    public boolean prevY = false;
    public boolean switchedDrive = false;

    public MovementGamePad(MecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
        this.gamepad = hardwareGamepad;
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public void updateRobot() {

        if(!switchedDrive) {
            robot.strafeR(
                    -gamepad.left_stick_x,
                    -gamepad.left_stick_y,
                    gamepad.right_stick_x
            );
        }else{
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad.left_stick_y,
                            -gamepad.left_stick_x,
                            -gamepad.right_stick_x
                    )
            );
        }

        if(gamepad.y && !prevY){
            switchedDrive = !switchedDrive;
        }

        prevY = gamepad.y;

        if(gamepad.dpad_up && !prevDpadUp){
            robot.lift.resetVerticalSlidePosition();
        }

        prevDpadUp = gamepad.dpad_up;

        if(gamepad.dpad_down && !prevDpadDown){
            robot.lift.resetHorizontalSlidePosition();
        }

        prevDpadDown = gamepad.dpad_down;

        if(gamepad.x && !prevX){
            robot.resetOdom();
        }

        if(gamepad.left_bumper){
            this.robot.lift.setHorizontalSlide(0);
        }

        prevLeftBump = gamepad.left_bumper;

        if(gamepad.right_bumper){
            this.robot.lift.setHorizontalSlide(MacroGamePad.horizontalTarget);
        }
        prevRightBump = gamepad.right_bumper;




    }
}
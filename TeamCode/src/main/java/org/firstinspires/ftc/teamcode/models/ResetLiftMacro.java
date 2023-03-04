package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class ResetLiftMacro implements Runnable {
    private final DcMotorEx lift;
    private final MecanumDrive robot;
    private final TouchSensor sensor;

    public boolean finished = false;

    public ResetLiftMacro(MecanumDrive robot, DcMotorEx lift, TouchSensor sensor) {
        this.robot = robot;
        this.lift = lift;
        this.sensor = sensor;
    }

    public void run() {
        this.robot.macroMode = true;
        this.lift.setPower(-0.5);
        this.lift.setTargetPosition(-3000);

        while (true) {
            if(this.sensor.isPressed()){
                this.lift.setPower(0);
                this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.robot.macroMode = false;
                break;
            }
        }
    }
}

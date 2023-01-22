package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.VERTICAL_LIFT_POWER;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem implements Subsystem {
    public DcMotorEx verticalLiftEncoder;
    public DcMotorEx horizontalSlide;

    public LiftSubsystem(HardwareMap hardwareMap) {
        verticalLiftEncoder = hardwareMap.get(DcMotorEx.class, "verticalLiftEncoder");

        verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetHorizontalSlidePosition() {
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetVerticalSlidePosition() {
        verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setVerticalLift(int position) {
        this.verticalLiftEncoder.setTargetPosition(position);
        this.verticalLiftEncoder.setPower(VERTICAL_LIFT_POWER);

        this.verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setVerticalMode(DcMotor.RunMode mode) {
        this.verticalLiftEncoder.setMode(mode);
    }

    public void setHorizontalSlide(int position) {
        this.horizontalSlide.setTargetPosition(position);
        this.horizontalSlide.setPower(HORIZONTAL_SLIDE_POWER);

        this.horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getHorizontalSlidePosition() {
        return this.horizontalSlide.getCurrentPosition();
    }

    public int getVerticalLiftPosition() {
        return this.verticalLiftEncoder.getCurrentPosition();
    }

}

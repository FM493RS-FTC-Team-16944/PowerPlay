package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HORIZONTAL_SLIDE_POWER;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.VERTICAL_LIFT_POWER;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem implements Subsystem {
    public final Telemetry telemetry;
    private final LiftPIDController verticalPID;
    private final LiftPIDController horizontalPID;
    public DcMotorEx verticalLiftEncoder;
    public DcMotorEx horizontalSlide;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        verticalLiftEncoder = hardwareMap.get(DcMotorEx.class, "verticalLiftEncoder");

        verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalLiftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide");

        this.telemetry = telemetry;
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalPID = new LiftPIDController(
                hardwareMap,
                horizontalSlide,
                1,
                1,
                1
        );

        verticalPID = new LiftPIDController(
                hardwareMap,
                verticalLiftEncoder,
                1,
                1,
                1
        );
    }

    public void loop() {
        verticalPID.loop();
        horizontalPID.loop();
    }

    public void resetHorizontalSlidePosition() {
        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetVerticalSlidePosition() {
        verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

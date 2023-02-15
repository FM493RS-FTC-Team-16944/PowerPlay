package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LiftPIDController {
    private MotionProfile profile;
    private final ElapsedTime voltageTimer;
    private final PIDController controller;
    private final VoltageSensor voltageSensor;
    private double voltage;
    private final ElapsedTime timer;
    private final DcMotorEx lift;
    private int endPos;
    private MotionState curState;
    private double targetPosition;

    public LiftPIDController(
            HardwareMap hardwareMap,
            DcMotorEx lift,
            double P,
            double I,
            double D
    ) {
        this.profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(1, 0),
                new MotionState(0, 0),
                30,
                25
        );

        this.lift = lift;

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.timer = new ElapsedTime();
        timer.reset();

        this.controller = new PIDController(P, I, D);
        controller.setPID(P, I, D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop() {
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        curState = profile.get(timer.time());
        if (curState.getV() != 0) {
            targetPosition = curState.getX();
        }

        this.lift.setPower(-controller.calculate(lift.getCurrentPosition(), targetPosition) / voltage * 14);
    }

    public void newProfile(double targetPos, double max_v, double max_a) {
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(lift.getCurrentPosition(), 0),
                new MotionState(targetPos, 0),
                max_v,
                max_a
        );

        endPos = (int) targetPos;
        timer.reset();
    }
}

package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TelemLog;

public class IntakeSubsystem implements Subsystem {
    public final Servo rotatorClaw;
    public final Servo leftClaw;
    public final Servo rightClaw;
    public final Servo tiltClaw;
    public final Servo armClaw;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        rotatorClaw = hardwareMap.get(Servo.class, "rotatorClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        tiltClaw = hardwareMap.get(Servo.class, "tiltClaw");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        this.armClaw.setDirection(Servo.Direction.REVERSE);
        this.tiltClaw.setDirection(Servo.Direction.REVERSE);

        this.leftClaw.setDirection(Servo.Direction.REVERSE);
    }

    public void openClaw() {//2 //8
        this.leftClaw.setPosition(OPEN_CLAW_POSITION);
        this.rightClaw.setPosition(OPEN_CLAW_POSITION);
    }
    //3 move horizontal

    public void closeClaw() {//4
        this.leftClaw.setPosition(CLOSE_CLAW_POSITION);
        this.rightClaw.setPosition(CLOSE_CLAW_POSITION);
    }

    public void setArmClawPosition(double position) {
        this.armClaw.setPosition(position);
    }

    public void setTiltClawPosition(double position) {
        this.tiltClaw.setPosition(position);
    }

    public void setRotatorClawPosition(double position) {
        this.rotatorClaw.setPosition(position);
    }

    public void groundIntake(double constant) {//1
        this.armClaw.setPosition(constant);
        this.tiltClaw.setPosition(NEUTRAL_CLAW_TILT_POSITION);
        this.rotatorClaw.setPosition(NORMAL_ROTATOR_POSITION);
    }

    public void hangingIntake() {//5
        this.armClaw.setPosition(DROP_ARM_CLAW_POSITION);
        this.tiltClaw.setPosition(UP_CLAW_TILT_POSITION);
        this.rotatorClaw.setPosition(NORMAL_ROTATOR_POSITION);
    }

    //6 retract horizontal

    public void rotatedHangingIntake() {//7
        this.armClaw.setPosition(0.41);
        this.tiltClaw.setPosition(DROP_CLAW_TILT_POSITION+0.1);
        this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
    }

    public void transferIntake() {
        this.armClaw.setPosition(0.46);
        this.tiltClaw.setPosition(DROP_CLAW_TILT_POSITION);
        this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
    }
}

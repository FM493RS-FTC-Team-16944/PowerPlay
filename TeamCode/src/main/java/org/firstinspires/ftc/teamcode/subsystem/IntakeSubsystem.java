package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_CONE_RETRIEVAL;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_CONE_RETRIEVAL_INVERTED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_CONE_RETRIEVAL_UP;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.AUX_ODOM_LOWERED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.AUX_ODOM_RETRACTED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.CLOSE_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION_B;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_ARM_CLAW_POSITION_C;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LEFT_ODOM_LOWERED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LEFT_ODOM_RETRACTED;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.TILT_CLAW_CONE_RETRIEVAL;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.TILT_CLAW_CONE_RETRIEVAL_UP;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem implements Subsystem {
    public final Servo rotatorClaw;
    public final Servo leftClaw;
    public final Servo rightClaw;
    public final Servo tiltClaw;
    public final Servo armClaw;

    public final Servo leftRetraction;
    public final Servo auxRetraction;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        rotatorClaw = hardwareMap.get(Servo.class, "rotatorClaw");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        tiltClaw = hardwareMap.get(Servo.class, "tiltClaw");
        armClaw = hardwareMap.get(Servo.class, "armClaw");

        leftRetraction = hardwareMap.get(Servo.class, "leftRetraction");
        auxRetraction = hardwareMap.get(Servo.class, "auxRetraction");

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
        this.armClaw.setPosition(DROP_ARM_CLAW_POSITION_B);
        this.tiltClaw.setPosition(DROP_CLAW_TILT_POSITION + 0.1);
        this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
    }

    public void transferIntake() {
        this.armClaw.setPosition(DROP_ARM_CLAW_POSITION_C);
        this.tiltClaw.setPosition(DROP_CLAW_TILT_POSITION);
        this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
    }

    public void retrievalIntakeUP(boolean inverted) {
        this.armClaw.setPosition(ARM_CLAW_CONE_RETRIEVAL_UP);
        this.tiltClaw.setPosition(TILT_CLAW_CONE_RETRIEVAL_UP);
        if (!inverted) {
            this.rotatorClaw.setPosition(NORMAL_ROTATOR_POSITION);
        } else {
            this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
        }
    }

    public void retrievalIntake(boolean inverted) {
        this.tiltClaw.setPosition(TILT_CLAW_CONE_RETRIEVAL);
        if (!inverted) {
            this.armClaw.setPosition(ARM_CLAW_CONE_RETRIEVAL);
            this.rotatorClaw.setPosition(NORMAL_ROTATOR_POSITION);
        } else {
            this.armClaw.setPosition(ARM_CLAW_CONE_RETRIEVAL_INVERTED);
            this.rotatorClaw.setPosition(ONE_EIGHTY_ROTATOR_POSITION);
        }
    }


    public void liftOdometry() {
        this.auxRetraction.setPosition(AUX_ODOM_RETRACTED);
        this.leftRetraction.setPosition(LEFT_ODOM_RETRACTED);
    }

    public void lowerOdometry(){
        this.auxRetraction.setPosition(AUX_ODOM_LOWERED);
        this.leftRetraction.setPosition(LEFT_ODOM_LOWERED);
    }
}

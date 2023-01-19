package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.DROP_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NEUTRAL_CLAW_TILT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.NORMAL_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ONE_EIGHTY_ROTATOR_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.UP_CLAW_TILT_POSITION;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class LiftMacro implements Runnable {
    private final double armClawPosition;
    private final boolean secondTilt;
    private MecanumDrive robot;

    public LiftMacro(MecanumDrive robot, double armClawPosition, boolean secondTilt) {
        this.robot = robot;
        this.armClawPosition = armClawPosition;
        this.secondTilt = secondTilt;
    }

    @Override
    public void run() {
        this.robot.openClaw();
        this.robot.setArmClawPosition(armClawPosition);

        this.robot.closeClaw();

        if(secondTilt) {
            this.robot.setTiltClawPosition(UP_CLAW_TILT_POSITION);
        }

        this.robot.setTiltClawPosition(NEUTRAL_CLAW_TILT_POSITION);
        this.robot.setArmClawPosition(0);

        this.robot.setRotatorClawPosition(ONE_EIGHTY_ROTATOR_POSITION);
        this.robot.setTiltClawPosition(DROP_CLAW_TILT_POSITION);

        this.robot.openClaw();

        this.robot.setTiltClawPosition(NEUTRAL_CLAW_TILT_POSITION);
        this.robot.setRotatorClawPosition(NORMAL_ROTATOR_POSITION);
    }
}

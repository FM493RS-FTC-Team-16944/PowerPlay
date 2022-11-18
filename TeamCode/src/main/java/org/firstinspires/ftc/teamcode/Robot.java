package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.util.TelemLog;

public class Robot {
    public final LinearOpMode teleOp;
    public final TelemLog telemetry;
    public final State state;
    public final RobotHardware hardware;
    public final RobotMovement movement;


    public Robot(LinearOpMode teleOp) {
        this.teleOp = teleOp;

        this.telemetry = new TelemLog(teleOp.telemetry);
        this.state = new State();
        this.hardware = new RobotHardware(this);
        this.movement = new RobotMovement(this);
    }

    public static class State {}
}

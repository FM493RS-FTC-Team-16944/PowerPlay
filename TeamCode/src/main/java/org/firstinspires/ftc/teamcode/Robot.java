package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TelemLog;

public class Robot {
    public final LinearOpMode teleOp;
    public final TelemLog telemetry;
    public final State state;
    public final RobotHardware hardware;
    public final RobotMovement movement;
    private final SampleMecanumDrive drive;


    public Robot(LinearOpMode teleOp) {
        this.teleOp = teleOp;

        this.telemetry = new TelemLog(teleOp.telemetry);
        this.state = new State();
        this.hardware = new RobotHardware(this);

        this.drive = new SampleMecanumDrive(this.teleOp.hardwareMap);
        this.movement = new RobotMovement(drive);
    }

    public static class State {}
}

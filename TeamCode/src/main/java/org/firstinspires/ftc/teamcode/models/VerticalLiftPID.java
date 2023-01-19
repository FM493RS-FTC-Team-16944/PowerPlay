package org.firstinspires.ftc.teamcode.models;


import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKD;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKI;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKP;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class VerticalLiftPID extends Thread{
    Telemetry telemetry;
    private MecanumDrive drive;
    private int height;
    public boolean complete;
    public int threshold;
    PIDController control;

    public VerticalLiftPID(MecanumDrive mecanumDrive, int height, int threshold, Telemetry tele){
        this.drive = mecanumDrive;
        this.height = height;
        this.control = new PIDController(verticalKP,verticalKI,verticalKD);
        control.setSetPoint(this.height);
        control.setTolerance(threshold);
        this.threshold = threshold;
        this.telemetry = tele;
    }

    @Override
    public void run() {
        try
        {
            while (!isInterrupted() && !complete)
            {
                // we record the Y values in the main class to make showing them in telemetry
                // easier.
                double command = control.calculate(drive.verticalLiftEncoder.getCurrentPosition());
                drive.verticalLiftEncoder.setPower(0.2 * command);
                if(control.atSetPoint()){
                    this.complete = true;
                }
            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        // an error occurred in the run loop.
        catch (Exception e) {e.printStackTrace();}
        telemetry.addLine("running macro");
    }
}

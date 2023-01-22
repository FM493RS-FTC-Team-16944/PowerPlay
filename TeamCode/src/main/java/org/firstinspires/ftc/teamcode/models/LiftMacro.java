package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class LiftMacro extends Thread{
    private MecanumDrive drive;

    public boolean complete;
    private Telemetry telemetry;

    public VerticalLiftPID poleHeightPID;
    public HorizontalLiftPID horizontalPID;
    Thread poleThread;
    Thread horizThread;
    int horizHeight;

    public VerticalLiftPID zeroHeightPID;
    Thread zeroHeightThread;
    public HorizontalLiftPID zeroHorizPID;
    Thread zeroHorizThread;
    public boolean done0 = false;
    public boolean done1 = false;
    public boolean done2 = false;
    public boolean done3 = false;
    public boolean done4 = false;


    public LiftMacro(MecanumDrive drive, int verticalHeight, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        poleHeightPID = new VerticalLiftPID(drive, verticalHeight+190, 200, telemetry);
        zeroHeightPID = new VerticalLiftPID(drive, -190, 200, telemetry);

    }


    public void run(){



        try
        {
            while (!isInterrupted() && !complete)
            {
                poleHeightPID.start();
                if (poleHeightPID.complete) {
                    try {
                        Thread.sleep(600);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    zeroHeightPID.start();
                    if (zeroHeightPID.complete) {
                        this.complete = true;
                    }
                }

            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        // an error occurred in the run loop.
        catch (Exception e) {e.printStackTrace();}


    }

}










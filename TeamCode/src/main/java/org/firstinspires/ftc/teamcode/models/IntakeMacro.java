package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class IntakeMacro extends Thread{
    private MecanumDrive drive;

    public boolean complete;
    private Telemetry telemetry;

    public HorizontalLiftPID horizontalPID;
    Thread horizThread;
    int horizHeight;

    public VerticalLiftPID zeroHeightPID;
    public HorizontalLiftPID zeroHorizPID;
    Thread zeroHorizThread;
    public boolean done0 = false;
    public boolean done1 = false;
    public boolean done2 = false;
    public boolean done3 = false;
    public boolean done4 = false;


    public IntakeMacro(MecanumDrive drive, int horizHeight, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.horizHeight = horizHeight;
        horizontalPID = new HorizontalLiftPID(drive, horizHeight, 200, telemetry);
        horizThread = new Thread(horizontalPID);
        zeroHorizPID = new HorizontalLiftPID(drive, -80, 100, telemetry);
    }


    public void run(){


        telemetry.addData("Horizontal Completed:", horizontalPID.complete);


        try
        {
            while (!isInterrupted() && !complete)
            {
                if(!done0) {
                    done0 = true;
                    MacroGamePad.ClawOpen = OpenClose.OPEN;
                    this.drive.intake.openClaw();
                    this.drive.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                horizontalPID.start();
                if(horizontalPID.complete) {
                    try{
                        Thread.sleep(500);
                    }catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    try{
                        Thread.sleep(1000);
                    }catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if(!done1) {
                        MacroGamePad.ClawOpen = OpenClose.CLOSE;
                        done1 = true;
                        this.drive.intake.closeClaw();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        this.drive.intake.hangingIntake();
                        this.drive.intake.transferIntake();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    zeroHorizPID.start();
                    if (zeroHorizPID.complete) {
                        if (!done2) {
                            try {
                                Thread.sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            MacroGamePad.ClawOpen = OpenClose.OPEN;
                            done2 = true;
                            this.drive.intake.openClaw();
                            try {
                                Thread.sleep(750);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            this.drive.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
                            try {
                                Thread.sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            this.complete = true;
                        }
                        telemetry.addLine("done going back to 0 horiz");
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



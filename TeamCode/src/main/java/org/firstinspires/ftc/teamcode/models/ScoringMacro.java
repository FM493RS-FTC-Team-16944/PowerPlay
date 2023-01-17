package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LOW_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.source.doctree.VersionTree;

import org.ejml.equation.Macro;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.operations.SplitTeleOP;

public class ScoringMacro extends Thread{
    private SampleMecanumDrive drive;

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


    public ScoringMacro(SampleMecanumDrive drive, int verticalHeight, int horizHeight, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.horizHeight = horizHeight;
        poleHeightPID = new VerticalLiftPID(drive, verticalHeight+190, 200, telemetry);
        poleThread = new Thread(poleHeightPID);
        horizontalPID = new HorizontalLiftPID(drive, horizHeight, 200, telemetry);
        horizThread = new Thread(horizontalPID);
        zeroHeightPID = new VerticalLiftPID(drive, -190, 200, telemetry);
        zeroHeightThread = new Thread(zeroHeightPID);
        zeroHorizPID = new HorizontalLiftPID(drive, -80, 100, telemetry);
        zeroHeightThread = new Thread(zeroHorizPID);
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
                    this.drive.openClaw();
                    this.drive.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
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
                    this.drive.closeClaw();
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    this.drive.hangingIntake();
                    this.drive.transferIntake();
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
                            this.drive.openClaw();
                            try {
                                Thread.sleep(750);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            this.drive.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
                            try {
                                Thread.sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }
                        telemetry.addLine("done going back to 0 horiz");
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
            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        // an error occurred in the run loop.
        catch (Exception e) {e.printStackTrace();}


    }

}



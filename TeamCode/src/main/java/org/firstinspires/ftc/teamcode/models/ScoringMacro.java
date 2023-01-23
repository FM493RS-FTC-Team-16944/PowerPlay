package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKD;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class ScoringMacro extends Thread {
    private MecanumDrive drive;

    public boolean complete;
    private Telemetry telemetry;

    public VerticalLiftPID poleHeightPID;
    public HorizontalLiftPID horizontalPID;
    Thread poleThread;
    Thread horizThread;
    int horizHeight;
    int verticalHeight;

    public VerticalLiftPID zeroHeightPID;
    Thread zeroHeightThread;
    public HorizontalLiftPID zeroHorizPID;
    Thread zeroHorizThread;
    public boolean done0 = false;
    public boolean done1 = false;
    public boolean done2 = false;
    public boolean done3 = false;
    public boolean done4 = false;


    public ScoringMacro(MecanumDrive drive, int verticalHeight, int horizHeight, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        this.horizHeight = horizHeight;
        this.verticalHeight = verticalHeight;
        poleHeightPID = new VerticalLiftPID(drive, verticalHeight + 190, 200, telemetry);
        poleThread = new Thread(poleHeightPID);
        horizontalPID = new HorizontalLiftPID(drive, horizHeight, 200, telemetry);
        horizThread = new Thread(horizontalPID);
        zeroHeightPID = new VerticalLiftPID(drive, -190, 200, telemetry);
        zeroHeightThread = new Thread(zeroHeightPID);
        zeroHorizPID = new HorizontalLiftPID(drive, -80, 100, telemetry);
        zeroHeightThread = new Thread(zeroHorizPID);
    }


    public void run() {


        telemetry.addData("Horizontal Completed:", horizontalPID.complete);


        try {
            while (!isInterrupted() && !complete) {
                if (!done0) {
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
                this.drive.lift.setHorizontalSlide(horizHeight);
                if (Math.abs(this.drive.lift.getHorizontalSlidePosition() - horizHeight) <= 50) {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    if (!done1) {
                        MacroGamePad.ClawOpen = OpenClose.CLOSE;
                        done1 = true;
                        this.drive.intake.closeClaw();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        this.drive.intake.rotatedHangingIntake();
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }
                    this.drive.lift.setHorizontalSlide(0);
                    if (this.drive.lift.getHorizontalSlidePosition() <= 50) {
                        if (!done2) {
                            try {
                                Thread.sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            MacroGamePad.ClawOpen = OpenClose.OPEN;
                            done2 = true;
                            this.drive.intake.transferIntake();
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
                        }
                        telemetry.addLine("done going back to 0 horiz");
                        this.drive.lift.setVerticalLift(verticalHeight);
                        if (Math.abs(this.drive.lift.getVerticalLiftPosition() - verticalHeight) <= 50) {
                            try {
                                Thread.sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            this.drive.lift.setVerticalLift(0);
                            if (this.drive.lift.getVerticalLiftPosition() <= 50) {
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
        catch (Exception e) {
            e.printStackTrace();
        }


    }

}



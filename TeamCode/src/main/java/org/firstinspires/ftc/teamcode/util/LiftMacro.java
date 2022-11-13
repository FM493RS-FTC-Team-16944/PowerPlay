package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;

public class LiftMacro implements Runnable {
    private MecanumDriveTrain driveTrain;

    public boolean complete = false;
    private final int aimHeight;
    private final String lift;

    public LiftMacro(int height, MecanumDriveTrain driveTrain, String lift) {
        this.aimHeight = height;
        this.driveTrain = driveTrain;

        this.lift = lift;
    }

    @Override
    public void run() {
        if (lift.equals("leftLift")) {
            driveTrain.leftLift.goToPosition(aimHeight, 0.3);
        } else {
            driveTrain.rightLift.goToPosition(aimHeight, 0.3);
        }

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if(aimHeight > 0) {
            if (lift.equals("leftLift")) {
                driveTrain.leftClaw.setPosition(0.75);
            } else {
                driveTrain.rightClaw.setPosition(0.75);
            }
        } else {
            if (lift.equals("leftLift")) {
                driveTrain.leftClaw.setPosition(0);
            } else {
                driveTrain.rightClaw.setPosition(0);
            }
        }

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (lift.equals("leftLift")) {
            driveTrain.leftClaw.setPosition(0);
        } else {
            driveTrain.rightClaw.setPosition(0);
        }

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (lift.equals("leftLift")) {
            driveTrain.leftLift.goToPosition(0, 0.3);
        } else {
            driveTrain.rightLift.goToPosition(0, 0.3);
        }

        this.complete = true;
    }
}

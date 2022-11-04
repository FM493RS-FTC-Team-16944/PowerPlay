package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.MecanumDriveTrain;

public class LiftMacro implements Runnable {
    RobotMovement movement;
    public boolean complete;
    int height;
    int aimHeight;
    public MecanumDriveTrain driveTrain;
    String lift;

    public LiftMacro(RobotMovement movement, int height, MecanumDriveTrain driveTrain, String lift) {
        this.movement = movement;
        this.aimHeight = height;
        this.driveTrain = driveTrain;
        this.lift = lift;
    }

    @Override
    public void run() {
        int deltaHeight = aimHeight - height;
        if(lift == "leftLift"){
            driveTrain.leftLift.goToPosition(deltaHeight, 0.5);
        }else{
            driveTrain.rightLift.goToPosition(deltaHeight, 0.5);
        }
        this.height = aimHeight;
    }
}

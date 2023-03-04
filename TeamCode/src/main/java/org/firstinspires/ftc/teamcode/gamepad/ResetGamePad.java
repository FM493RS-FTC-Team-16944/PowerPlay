package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;

import org.firstinspires.ftc.teamcode.models.GrabPosition;
import org.firstinspires.ftc.teamcode.models.NewScoreMacro;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.models.ResetLiftMacro;

public class ResetGamePad {
    private final MecanumDrive robot;
    private final Gamepad gamepad;

    private final boolean macroMode = false;


    private boolean previousA = false;
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousBumpRight = false;
    private boolean previousBumpLeft = false;
    private boolean previousPadRight = false;
    private boolean previousPadLeft = false;
    private boolean highPole = false;
    private boolean mediumPole = false;


    private int intakePosition = 19;
    private int retrievalIndex = 1;
    private boolean invertedRetrieval = false;
    private boolean retrievingCone = false;
    private DcMotorEx selectedLift;
    NewScoreMacro scoringMac;
    ResetLiftMacro vertMac;
    ResetLiftMacro horizMac;



    public ResetGamePad(MecanumDrive robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
        this.robot.lift.verticalLiftEncoder.setTargetPosition(0);
        this.scoringMac = new NewScoreMacro(
                robot, new GrabPosition(0.005 , 1500)
        );
        this.vertMac = new ResetLiftMacro(
                robot,this.robot.lift.verticalLiftEncoder, this.robot.lift.verticalLimitSwitch
        );
        this.horizMac = new ResetLiftMacro(robot,this.robot.lift.horizontalSlide, this.robot.lift.horizontalLimitSwitch);
    }

    public void updateRobot() {


        if (gamepad.x && gamepad.x != previousX) {
            robot.resetOdom();
            //robot.lift.resetLifts();
        }
        previousX = gamepad.x;



        if(gamepad.y && !previousY){
            Thread thread = new Thread(scoringMac);
            thread.start();
        }
        previousY = gamepad.y;

        if(gamepad.a && !previousA){
            Thread vertThread = new Thread(vertMac);
            vertThread.start();
            Thread horizThread = new Thread(horizMac);
            horizThread.start();
        }
        previousA = gamepad.a;

    }


}

package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LOW_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.models.HorizontalLiftPID;
import org.firstinspires.ftc.teamcode.models.Lift;
import org.firstinspires.ftc.teamcode.models.Mode;
import org.firstinspires.ftc.teamcode.models.OpenClose;
import org.firstinspires.ftc.teamcode.models.ScoringMacro;
import org.firstinspires.ftc.teamcode.models.VerticalLiftPID;
import org.firstinspires.ftc.teamcode.operations.SplitTeleOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MacroGamePad {
    private final Gamepad gamepad;
    private final MecanumDrive robot;
    private final Telemetry telemetry;

    public static int armClawPosition = 1;
    public static int rotatorPosition = 1;
    public static int clawTiltPosition = 1;
    public static int horizontalSlidePosition = 1;
    public static int verticalLiftPosition = 1;

    private int position = 3;
    private int lift = 1;
    private int slide = 1;
    private double vPower = 0;

    public static OpenClose ClawOpen = OpenClose.CLOSE;
    OpenClose rightClawOpen = OpenClose.CLOSE;


    public Motor currentLift;
    public boolean prevSwitched = false;

    public boolean prevRightClaw = false;
    public boolean prevClaw = false;
    public boolean prevLeftLift = false;
    public boolean prevRightLift = false;
    public boolean prevA = false;
    public boolean prevB = false;
    public boolean prevY = false;
    private boolean prevX = false;
    private boolean prevLeftBump = false;
    private boolean prevRightBump = false;
    private boolean prevLeftTrig = false;
    private boolean prevRightTrig = false;
    private boolean justWentDown = false;
    ScoringMacro scoringMac;
    Thread scoringThread;

    HorizontalLiftPID testHor;
    Thread horizThread;
    ScoringMacro currentThread;

    public static int horizontalTarget = 2000;
    private int vertHeight = 1600;


    public MacroGamePad(MecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
        this.gamepad = hardwareGamepad;
        this.robot = robot;
        this.telemetry = telemetry;
        this.scoringMac = new ScoringMacro(robot,vertHeight, horizontalTarget,telemetry);
        this.scoringThread = new Thread(scoringMac);
        this.scoringMac.complete = true;
        this.testHor = new HorizontalLiftPID(robot,1000, 100, telemetry);
        horizThread = new Thread(testHor);
    }

    public void updateRobot() {
        SplitTeleOP.currentThread = this.currentThread;
        telemetry.addData("Vertical Lift Position:", this.robot.lift.getVerticalLiftPosition());
        telemetry.addData("Horizontal Lift Position:", this.robot.lift.getHorizontalSlidePosition());

        telemetry.addData("Vertical Target:", vertHeight);
        telemetry.addData("Horizontal Target:", horizontalTarget);

        if(this.robot.lift.getVerticalLiftPosition() <= 100 && justWentDown){
            justWentDown = false;
            this.robot.lift.resetVerticalSlidePosition();
        }

        if(gamepad.y && !prevY){
            telemetry.addData("Horizontal Target:", robot.lift.getHorizontalSlidePosition());
            horizontalTarget = robot.lift.getHorizontalSlidePosition();
            vertHeight = robot.lift.getVerticalLiftPosition();
            telemetry.update();
        }

        if(gamepad.left_trigger > 0.3 && !prevLeftTrig){
            vertHeight = HIGH_SCORE_VERTICAL_LIFT_POSITION;
        }

        prevLeftTrig = (gamepad.left_trigger > 0.3);


        if(gamepad.left_bumper && !prevLeftBump){
            robot.lift.setVerticalLift(0);
            telemetry.addLine("zero height reached");
            justWentDown = true;
            telemetry.update();
        }

        prevLeftBump = gamepad.left_bumper;

        if(gamepad.right_bumper && !prevRightBump){
            robot.lift.setVerticalLift(vertHeight);
            telemetry.addLine("target height reached");
            telemetry.update();
        }
        prevRightBump = gamepad.right_bumper;

        prevY = gamepad.y;


        if (gamepad.dpad_up) {
            vertHeight+=5;
        }

        if (gamepad.dpad_down) {
            vertHeight-=5;
        }

        if (gamepad.dpad_right) {
            horizontalTarget+=5;
        }

        if (gamepad.dpad_left) {
            horizontalTarget-=5;
        }



        if(gamepad.x && !prevX){
            telemetry.addLine("scoring thread started");
            scoringMac = new ScoringMacro(robot, vertHeight, horizontalTarget,telemetry);
            scoringMac.start();
            currentThread = scoringMac;
//            scoringThread.start();
//            telemetry.update();
        }
        prevX = gamepad.x;

        if (gamepad.b && gamepad.b != prevB) {
            position++;
            position %= 4;
            if (position==0) {
                this.robot.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            } else if (position == 1) {
                this.robot.intake.hangingIntake();
            } else if (position == 2) {
                this.robot.intake.rotatedHangingIntake();
            } else if (position == 3) {
                this.robot.intake.transferIntake();
            }
        }
        prevB = gamepad.b;

        if (gamepad.a && gamepad.a != prevA) {
            if (ClawOpen == OpenClose.OPEN) {
                ClawOpen = OpenClose.CLOSE;
                this.robot.intake.closeClaw();
            } else {
                ClawOpen = OpenClose.OPEN;
                this.robot.intake.openClaw();
            }
        }
        prevA = gamepad.a;



    }
}
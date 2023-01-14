package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.LOW_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotMovement;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
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
    private final SampleMecanumDrive robot;
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
    ScoringMacro scoringMac;
    Thread scoringThread;

    HorizontalLiftPID testHor;
    Thread horizThread;
    ScoringMacro currentThread;

    private int horizontalTarget = 2000;
    private int vertHeight = 170;
    private int verticalTarget = HIGH_SCORE_VERTICAL_LIFT_POSITION;

    public MacroGamePad(SampleMecanumDrive robot, Gamepad hardwareGamepad, Telemetry telemetry) {
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
        telemetry.addData("Horizontal Slide Position:", robot.getHorizontalSlidePosition()) ;
        telemetry.addData("Vertical Slide Position:", robot.getVerticalLiftPosition()


        ) ;
        if(gamepad.y && !prevY){
            telemetry.addData("Horizontal Target:", robot.getHorizontalSlidePosition());
            horizontalTarget = robot.getHorizontalSlidePosition();
            vertHeight = robot.getVerticalLiftPosition();
            telemetry.update();
        }

        if(gamepad.left_trigger > 0.3 && !prevLeftTrig){
            verticalTarget = HIGH_SCORE_VERTICAL_LIFT_POSITION;
        }

        prevLeftTrig = (gamepad.left_trigger > 0.3);


        if(gamepad.left_bumper && !prevLeftBump){
            verticalTarget = MEDIUM_SCORE_VERTICAL_LIFT_POSITION;
            telemetry.addLine("Medium score set");
            telemetry.update();
        }

        prevLeftBump = gamepad.left_bumper;

        if(gamepad.right_bumper && !prevRightBump){
            verticalTarget = LOW_SCORE_VERTICAL_LIFT_POSITION;
        }

        prevY = gamepad.y;


        if (gamepad.dpad_up) {
            this.robot.verticalLiftEncoder.setPower(0.6);
        }else if (gamepad.dpad_down) {
            this.robot.verticalLiftEncoder.setPower(-0.5);
        }else{
            this.robot.verticalLiftEncoder.setPower(0);
        }
        if (gamepad.dpad_right) {
            this.robot.horizontalSlide.setPower(1);
        }else if (gamepad.dpad_left) {
            this.robot.horizontalSlide.setPower(-1);
        }else{
            this.robot.horizontalSlide.setPower(0);
        }



        if(gamepad.x && !prevX){
            telemetry.addLine("scoring thread started");
            scoringMac = new ScoringMacro(robot,vertHeight, horizontalTarget,telemetry);
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
                this.robot.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            } else if (position == 1) {
                this.robot.hangingIntake();
            } else if (position == 2) {
                this.robot.rotatedHangingIntake();
            } else if (position == 3) {
                this.robot.transferIntake();
            }
        }
        prevB = gamepad.b;

        if (gamepad.a && gamepad.a != prevA) {
            if (ClawOpen == OpenClose.OPEN) {
                ClawOpen = OpenClose.CLOSE;
                this.robot.closeClaw();
            } else {
                ClawOpen = OpenClose.OPEN;
                this.robot.openClaw();
            }
        }
        prevA = gamepad.a;



    }
}
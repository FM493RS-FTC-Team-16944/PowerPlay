package org.firstinspires.ftc.teamcode.gamepad;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIRST_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FOURTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_SECOND_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_THIRD_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.HIGH_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.MEDIUM_SCORE_VERTICAL_LIFT_POSITION;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.OPEN_CLAW_POSITION;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class RandomGamePad {
    private final MecanumDrive robot;
    private final Gamepad gamepad;




    private boolean previousA = false;
    private boolean previousB = false;
    private boolean previousX = false;
    private boolean previousY = false;
    private boolean previousRight = false;
    private boolean previousLeft = false;


    public RandomGamePad(MecanumDrive robot, Gamepad gamepad) {
        this.robot = robot;
        this.gamepad = gamepad;
    }

    public void updateRobot() {

        if (gamepad.a && gamepad.a != previousA) {
            robot.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
        }
        previousA = gamepad.a;

        if (gamepad.b && gamepad.b != previousB) {
            robot.intake.groundIntake(ARM_CLAW_POSITION_FOURTH_CONE);
        }
        previousB = gamepad.b;

        if (gamepad.x && gamepad.x != previousX) {
            robot.intake.groundIntake(ARM_CLAW_POSITION_THIRD_CONE);
        }
        previousX = gamepad.x;

        if (gamepad.y && gamepad.y != previousY) {
            robot.intake.groundIntake(ARM_CLAW_POSITION_SECOND_CONE);
        }
        previousY = gamepad.y;

        if (gamepad.right_bumper && gamepad.right_bumper != previousRight) {
            robot.intake.groundIntake(ARM_CLAW_POSITION_FIRST_CONE);
        }
        previousRight = gamepad.right_bumper;





    }


}

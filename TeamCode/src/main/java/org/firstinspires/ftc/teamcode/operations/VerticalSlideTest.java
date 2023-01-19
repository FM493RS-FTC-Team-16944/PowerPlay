package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKD;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKI;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@TeleOp(name = "VerticalSlideTest")
public class VerticalSlideTest extends LinearOpMode {
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap);
        MovementGamePad gamePad = new MovementGamePad(drive, this.gamepad1, this.telemetry);
        boolean dpadUP = false;
        PIDController control = new PIDController(verticalKP,verticalKI,verticalKD);
        double position = -2000;
        drive.verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.verticalLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        control.setSetPoint(position);
        while (opModeIsActive() && !isStopRequested()) {
            drive.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            gamePad.updateRobot();
            double command = control.calculate(drive.verticalLiftEncoder.getCurrentPosition());

            drive.verticalLiftEncoder.setPower(0.2 * command);
            //dpadUP = gamepad1.dpad_up;
            telemetry.addData("Vertical Position:", drive.verticalLiftEncoder.getCurrentPosition());
            drive.odometry.update();
            drive.outputOdomReadings(telemetry);
            telemetry.update();


        }
    }
}

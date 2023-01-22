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
        double position = 1500;
        drive.lift.verticalLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift.verticalLiftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.lift.verticalLiftEncoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        control.setSetPoint(position);
        while (opModeIsActive() && !isStopRequested()) {
            drive.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            gamePad.updateRobot();
            double command = control.calculate(drive.lift.verticalLiftEncoder.getCurrentPosition());

            drive.lift.verticalLiftEncoder.setPower(0.2 * command);
            //dpadUP = gamepad1.dpad_up;
            telemetry.addData("Vertical Position:", drive.lift.verticalLiftEncoder.getCurrentPosition());
            drive.odometry.update();
            drive.outputOdomReadings(telemetry);
            telemetry.update();


        }
    }
}

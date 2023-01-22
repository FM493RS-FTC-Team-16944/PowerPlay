package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKD;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKI;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

@TeleOp(name = "SlideTestsTeleOp")
public class SlideTestsTeleOp extends LinearOpMode {

    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        MovementGamePad gamePad = new MovementGamePad(drive, this.gamepad1, this.telemetry);
        boolean dpadUP = false;
        PIDController control = new PIDController(horizontalKP,horizontalKI,horizontalKD);
        double position = -2000;
        drive.lift.horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.lift.horizontalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.lift.horizontalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        control.setSetPoint(position);
        while (opModeIsActive() && !isStopRequested()) {
            drive.intake.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            gamePad.updateRobot();
            double command = control.calculate(drive.lift.horizontalSlide.getCurrentPosition());

            drive.lift.horizontalSlide.setPower(0.4 * command);
            //dpadUP = gamepad1.dpad_up;
            telemetry.addData("Horizontal Position:", drive.lift.horizontalSlide.getCurrentPosition());
            drive.odometry.update();
            drive.outputOdomReadings(telemetry);
            telemetry.update();


        }
    }
}

package org.firstinspires.ftc.teamcode.operations;

import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.ARM_CLAW_POSITION_FIFTH_CONE;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKD;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKI;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.horizontalKP;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKD;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKI;
import static org.firstinspires.ftc.teamcode.hardware.ArmConstants.verticalKP;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.gamepad.CombinedGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MacroGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ManualGamePad;
import org.firstinspires.ftc.teamcode.gamepad.MovementGamePad;
import org.firstinspires.ftc.teamcode.gamepad.TestGamePad;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

@TeleOp(name = "SlideTestsTeleOp")
public class SlideTestsTeleOp extends LinearOpMode {

    public Robot robot;

    public RobotHardware hardware;
    public ManualGamePad gamepad;

    @Override
    public void runOpMode() {
        waitForStart();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        MovementGamePad gamePad = new MovementGamePad(drive, this.gamepad1, this.telemetry);
        boolean dpadUP = false;
        PIDController control = new PIDController(horizontalKP,horizontalKI,horizontalKD);
        double position = 2700;
        control.setSetPoint(position);
        while (opModeIsActive() && !isStopRequested()) {
            drive.groundIntake(ARM_CLAW_POSITION_FIFTH_CONE);
            gamePad.updateRobot();
            double command = control.calculate(drive.horizontalSlide.getCurrentPosition());

            drive.horizontalSlide.setPower(0.2* command);
            //dpadUP = gamepad1.dpad_up;
            telemetry.addData("Horizontal Position:", drive.horizontalSlide.getCurrentPosition());
            drive.odometry.update();
            drive.outputOdomReadings(telemetry);
            telemetry.update();


        }
    }
}

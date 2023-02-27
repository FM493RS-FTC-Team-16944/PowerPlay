package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamepad.DriveGamePad;
import org.firstinspires.ftc.teamcode.gamepad.NewGamePad;
import org.firstinspires.ftc.teamcode.gamepad.RandomGamePad;
import org.firstinspires.ftc.teamcode.gamepad.ResetGamePad;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, this.telemetry);
        DriveGamePad gamePad = new DriveGamePad(drive, this.gamepad1);
        ResetGamePad gamePad2 = new ResetGamePad(drive,this.gamepad2);

        drive.setPoseEstimate(PoseStorage.currentPos);

        drive.odometry.liftOdometry();

        while (opModeIsActive() && !isStopRequested()) {
            gamePad.updateRobot();
            gamePad2.updateRobot();
            
            this.telemetry.addData("angle", drive.odometry.getHeading());
            this.telemetry.addData("vertical lift position", drive.lift.getVerticalLiftPosition());
            this.telemetry.addData("horizontal slide position", drive.lift.getHorizontalSlidePosition());
            this.telemetry.addData("Horizontal Limit Switched Pressed: ", drive.lift.horizontalLimitSwitch.isPressed());
            this.telemetry.addData("Vertical Limit Switched Pressed: ", drive.lift.verticalLimitSwitch.isPressed());
            telemetry.update();


            drive.updateLifts();
        }
    }
}

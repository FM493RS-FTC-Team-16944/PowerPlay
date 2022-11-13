package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.util.TelemLog;


@Config
public class MecanumDriveTrain {
    public final Motor topLeft;
    public final Motor backLeft;
    public final Motor topRight;
    public final Motor backRight;
    public final Motor leftLift;
    public final Motor rightLift;
    public final Motor leftEncoder;
    public final Motor rightEncoder;
    public final Motor auxEncoder;
    public final ServoMotor leftClaw;
    public final ServoMotor rightClaw;
    private final TelemLog telemetry;

    public static final double NEW_P = 2.5;
    public static final double NEW_I = 0.1;
    public static final double NEW_D = 0.2;

    public MecanumDriveTrain(
            String topLeftName,
            String backLeftName,
            String topRightName,
            String backRightName,
            String leftLiftName,
            String rightLiftName,
            String leftEncoderName,
            String rightEncoderName,
            String auxEncoderName,
            String leftClawName,
            String rightClawName,
            HardwareMap hardwareMap,
            TelemLog telemetry
    ) {
        this.topRight = new Motor(topRightName, hardwareMap, DcMotor.Direction.FORWARD);
        this.topLeft = new Motor(topLeftName, hardwareMap, DcMotor.Direction.REVERSE);
        this.backLeft = new Motor(backLeftName, hardwareMap, DcMotor.Direction.REVERSE);
        this.backRight = new Motor(backRightName, hardwareMap, DcMotor.Direction.FORWARD);

        this.rightLift = new Motor(rightLiftName, hardwareMap, DcMotor.Direction.FORWARD);
        this.leftLift = new Motor(leftLiftName, hardwareMap, DcMotor.Direction.FORWARD);

        this.rightEncoder = new Motor(rightEncoderName, hardwareMap, DcMotor.Direction.FORWARD);
        this.leftEncoder = new Motor(leftEncoderName, hardwareMap, DcMotor.Direction.REVERSE);
        this.auxEncoder = new Motor(auxEncoderName, hardwareMap, DcMotor.Direction.FORWARD);

        this.rightClaw = new ServoMotor(rightClawName, hardwareMap);
        this.leftClaw = new ServoMotor(leftClawName, hardwareMap);

        // You can call the methods to set the motor modes
        this.leftEncoder.reset();
        this.rightEncoder.reset();
        this.auxEncoder.reset();

        this.stop();

        setPIDCoefficients();

        // This part is depends on your preference for zero power behavior. For the implementation on my robot, I will set it to break mode.
        this.topLeft.setBreakMode();
        this.backLeft.setBreakMode();
        this.topRight.setBreakMode();
        this.backRight.setBreakMode();

        this.telemetry = telemetry;
    }

    public void setPIDCoefficients() {
        DcMotorEx rightLift = (DcMotorEx) this.rightLift;
        DcMotorEx leftLift = (DcMotorEx) this.rightLift;

        PIDFCoefficients newPIDCoefficients = new PIDFCoefficients(
                new PIDCoefficients(NEW_P, NEW_I ,NEW_D)
        );

        rightLift.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, newPIDCoefficients);
        leftLift.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER, newPIDCoefficients);
    }

    public void stop() {
        this.topRight.setPower(0);
        this.topLeft.setPower(0);
        this.backRight.setPower(0);
        this.backLeft.setPower(0);
    }

    public void updateEncoderReadings() {
        String leftEncoderT = RobotHardware.DECIMAL_FORMAT.format(leftEncoder.getCurrentPosition());
        String rightEncoderT = RobotHardware.DECIMAL_FORMAT.format(rightEncoder.getCurrentPosition());
        String auxEncoderT = RobotHardware.DECIMAL_FORMAT.format(auxEncoder.getCurrentPosition());

        telemetry.addData("Left Encoder Position Centimeters : " , leftEncoderT);
        telemetry.addData("Right Encoder Position Centimeters : " , rightEncoderT);
        telemetry.addData("Auxiliary Encoder Position Centimeters : " , auxEncoderT);
    }

    public void resetDriveEncoders() {
        setEncodersMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setEncodersMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setEncodersMode(DcMotor.RunMode mode) {
        leftLift.setMode(mode);
        rightLift.setMode(mode);
        leftEncoder.setMode(mode);
    }
}

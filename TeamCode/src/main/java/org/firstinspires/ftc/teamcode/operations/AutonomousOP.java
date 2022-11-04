package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.movement.ImageCommand;
import org.firstinspires.ftc.teamcode.movement.TrackingPathFinder;

import java.lang.reflect.InvocationTargetException;
import java.util.List;

@Autonomous
public class AutonomousOP extends LinearOpMode {
    public Robot robot;
    private TrackingPathFinder trackingPathFinder;

    @Override
    public void runOpMode() {
        this.robot = new Robot(this);
        this.trackingPathFinder = new TrackingPathFinder(this);

        waitForStart();

        while(opModeIsActive()) {
            List<String> detectedObjects = this.robot.hardware.detector.getObjects();

            for (String label : detectedObjects) {
                try {
                    this.trackingPathFinder.invoke(label);
                } catch (InvocationTargetException | IllegalAccessException e) {
                    e.printStackTrace();
                }
            }

            this.telemetry.update();
        }
    }

    @ImageCommand(name="1 Bolt")
    public void boltPath() {
        this.telemetry.addData("Object", "Bolt");
    }

    @ImageCommand(name="2 Bulb")
    public void bulbPath() {
        this.telemetry.addData("Object", "Bulb");
    }

    @ImageCommand(name="3 Panel")
    public void panelPath() {
        this.telemetry.addData("Object", "Panel");
    }
}

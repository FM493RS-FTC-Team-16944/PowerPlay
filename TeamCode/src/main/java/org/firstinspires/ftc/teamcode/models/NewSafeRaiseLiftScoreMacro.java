package org.firstinspires.ftc.teamcode.models;

import org.firstinspires.ftc.teamcode.hardware.ArmConstants;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class NewSafeRaiseLiftScoreMacro implements Runnable {
    private final GrabPosition state;
    private final MecanumDrive robot;
    private final Thread extendOutRunnable;
    private final Thread transferRunnable;

    public boolean finished = false;

    public NewSafeRaiseLiftScoreMacro(MecanumDrive robot, GrabPosition state) {
        this.robot = robot;
        this.state = state;

        this.extendOutRunnable = new Thread(
                new ExtendOutMacro(robot, state)
        );
        this.transferRunnable = new Thread(
                new TransferMacro(robot, state)
        );
    }

    public void run() {
        this.robot.lift.setVerticalLift(state.verticalPos);

//        if (state.horizontalPos > 700)
//            this.robot.lift.setHorizontalSlide(state.horizontalPos - 700);

//        if ((state.horizontalPos - 700) > 1200) {
//            while (true) {
//                if (robot.lift.getHorizontalSlidePosition() > 1000) {
//                    break;
//                }
//            }
//            robot.lift.activateSlideSupport();
//        }


        while (true) {
            if (robot.lift.getVerticalLiftPosition() <= state.verticalPos + 6 &&
                    robot.lift.getVerticalLiftPosition() >= state.verticalPos - 6)
                break;
        }

        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        this.robot.lift.setVerticalLift(ArmConstants.NEUTRAL_VERTICAL_LIFT_POSITION);

//        while (true) {
//            if (robot.lift.getVerticalLiftPosition() <= NEUTRAL_VERTICAL_LIFT_POSITION + 6 &&
//                    robot.lift.getVerticalLiftPosition() >= NEUTRAL_VERTICAL_LIFT_POSITION - 6)
//                break;
//        }

    }
}
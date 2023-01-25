package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.CYCLE_GRAB_POSITIONS;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class MacroManager {
    public final NewScoreMacro[] macroList = {};
    public final Thread[] threadList = {};
    public int index = 0;

    public MacroManager(MecanumDrive drive) {
        macroList[0] = new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[0]);
        macroList[1] = new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[1]);
        macroList[2] = new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[2]);
        macroList[3] = new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[3]);
        macroList[4] = new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[4]);

        threadList[0] = new Thread(macroList[0]);
        threadList[1] = new Thread(macroList[1]);
        threadList[2] = new Thread(macroList[2]);
        threadList[3] = new Thread(macroList[3]);
        threadList[4] = new Thread(macroList[4]);
    }

    public void startScoring() {
        if (index > threadList.length - 1) {
            return;
        }

        NewScoreMacro macro = macroList[index];

        if (!threadList[index].isAlive()) {
            threadList[index].start();
        }

        if (macro.finished) {
            index++;
            if (index < threadList.length) {
                threadList[index].start();
            }
        }
    }

    public boolean isFinished() {
        return index > threadList.length - 1;
    }
}

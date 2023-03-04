package org.firstinspires.ftc.teamcode.models;

import static org.firstinspires.ftc.teamcode.hardware.MecanumDrive.CYCLE_GRAB_POSITIONS;

import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;

public class MacroManager {
    public final NewScoreMacro[] macroList;
    public final Thread[] threadList;
    public final Thread[] threadSafeList;
    private final NewSafeScoreMacro[] safeMacroList;
    public int index = 0;

    public MacroManager(MecanumDrive drive) {
        macroList = new NewScoreMacro[]{
                new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[0]),
                new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[1]),
                new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[2]),
                new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[3]),
                new NewScoreMacro(drive, CYCLE_GRAB_POSITIONS[4])
        };

        safeMacroList = new NewSafeScoreMacro[]{
                new NewSafeScoreMacro(drive, CYCLE_GRAB_POSITIONS[0]),
                new NewSafeScoreMacro(drive, CYCLE_GRAB_POSITIONS[1]),
                new NewSafeScoreMacro(drive, CYCLE_GRAB_POSITIONS[2]),
                new NewSafeScoreMacro(drive, CYCLE_GRAB_POSITIONS[3]),
                new NewSafeScoreMacro(drive, CYCLE_GRAB_POSITIONS[4])
        };

        threadList = new Thread[]{
                new Thread(macroList[0]),
                new Thread(macroList[1]),
                new Thread(macroList[2]),
                new Thread(macroList[3]),
                new Thread(macroList[4])
        };

        threadSafeList = new Thread[]{
                new Thread(safeMacroList[0]),
                new Thread(safeMacroList[1]),
                new Thread(safeMacroList[2]),
                new Thread(safeMacroList[3]),
                new Thread(safeMacroList[4])
        };
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

    public void startSafeScoring() {
        if (index > threadSafeList.length - 1) {
            return;
        }

        NewSafeScoreMacro macro = safeMacroList[index];

        if (!threadSafeList[index].isAlive()) {
            threadSafeList[index].start();
        }

        if (macro.finished) {
            index++;
            if (index < threadSafeList.length) {
                threadSafeList[index].start();
            }
        }
    }

    public boolean isFinished() {
        return index > threadList.length - 1;
    }
}

package org.firstinspires.ftc.teamcode.util;

import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public class TelemLog {
    private final String BASE_FOLDER_NAME = "FIRST";

    public final Telemetry telemetry;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final Telemetry dashboardTelemetry = dashboard.getTelemetry();
    private final Logger log;
    private final MultipleTelemetry multipleTelemetry;

    public TelemLog(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.multipleTelemetry = new MultipleTelemetry(this.telemetry, this.dashboardTelemetry);

        this.log = Logger.getLogger("TelemLog");
        setupLogFile();
    }

    private void setupLogFile() {
        try {
            // This block configure the logger with handler and formatter
            String directoryPath = Environment.getExternalStorageDirectory().getPath() + "/" + BASE_FOLDER_NAME;

            FileHandler fh = new FileHandler(directoryPath + "/" + "TelemLog.log", true);
            this.log.addHandler(fh);

            SimpleFormatter formatter = new SimpleFormatter();
            fh.setFormatter(formatter);
        } catch (SecurityException | IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String msg, Object value, Level level) {
        this.multipleTelemetry.addData(msg, value);
        this.log.info(msg + value);
    }

    public void addData(String msg, Object value) {
        this.multipleTelemetry.addData(msg, value);
        this.log.info(msg + value);
    }

    public void addLine(String msg) {
        this.multipleTelemetry.addLine(msg);
        this.log.info(msg);
    }

    public void update() {
        this.multipleTelemetry.update();
    }
}

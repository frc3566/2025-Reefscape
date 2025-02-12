package frc.lib.util;

import java.text.SimpleDateFormat;
import java.util.Calendar;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RobotLogger {
    private static RobotLogger _logger;

    public static RobotLogger getLogger() { 
        if(_logger == null) {
            _logger = new RobotLogger();
        }
        return _logger;
    }

    public RobotLogger() {
        DataLogManager.start();
    }

    public void logString(LogLevel level, String category, String message) {
        if(level == LogLevel.SimulationOnly && !Robot.isSimulation()) {
            // Don't log simulation only messages if not in simulation
            return;
        }

        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
        String logMessage = String.format("%s | [%s] %s: %s", timeStamp, level.toString(), category, message);
        DataLogManager.log(logMessage);

        SmartDashboard.putString(level.toString(), logMessage);
    }

    public enum LogLevel {
        SimulationOnly,
        Debug,
        Warning,
        Info,
        Error
    }
}

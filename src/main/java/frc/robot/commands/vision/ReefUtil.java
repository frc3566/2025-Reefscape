package frc.robot.commands.vision;

import java.util.List;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class ReefUtil {
    public static final double adjustY = Units.inchesToMeters(6.469);
    public enum LeftRight {
        LEFT, RIGHT
    }

    public enum BranchLevel {
        TROUGH, L2, L3, L4
    }

    public static List<Integer> getTargettingIds() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            return List.of(17, 18, 19, 20, 21, 22);
        } else {
            return List.of(6, 7, 8, 9, 10, 11);
        }
    }
}
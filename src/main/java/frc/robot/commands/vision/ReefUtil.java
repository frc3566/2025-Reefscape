package frc.robot.commands.vision;

import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class ReefUtil {
    public static final double adjustY = Units.inchesToMeters(6.469);
    public enum LeftRight {
        LEFT, RIGHT
    }

    public enum Side {
        DS(new Pair<>(18, 7)), 
        DSLEFT(new Pair<>(19, 6)), 
        DSRIGHT(new Pair<>(17, 8)), 
        BARGE(new Pair<>(21, 10)), 
        BARGELEFT(new Pair<>(20, 11)), 
        BARGERIGHT(new Pair<>(22, 9));

        private Pair<Integer, Integer> targettingIds;
        
        private Side(Pair<Integer, Integer> targettingIds) {
            this.targettingIds = targettingIds;
        }

        public int getTargettingId() {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                return targettingIds.getFirst();
            } else {
                return targettingIds.getSecond();
            }
        }
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
package frc.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

/** Sets motor usage for a Spark Max motor controller */
public class SparkMaxUtil {
  public enum Usage {
    kAll,
    kPositionOnly,
    kVelocityOnly,
    kMinimal
  };

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a SparkMax is
   *     constructed.
   * @param enableFollowing Whether to enable motor following.
   */
  public static void setSparkMaxBusUsage(
      SparkMax motor, Usage usage, boolean enableFollowing) {
    if (enableFollowing) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 10);
    } else {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 500);
    }

    if (usage == Usage.kAll) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 50);
    } else if (usage == Usage.kPositionOnly) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    } else if (usage == Usage.kVelocityOnly) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    } else if (usage == Usage.kMinimal) {
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 500);
      motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus3, 500);
    }
  }

  /**
   * This function allows reducing a Spark Max's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 20ms to 500ms.
   *
   * <p>See
   * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
   * for a description of the status frames.
   *
   * @param motor The motor to adjust the status frame periods on.
   * @param usage The status frame feedack to enable. kAll is the default when a SparkMax is
   *     constructed.
   */
  public static void setSparkMaxBusUsage(SparkMax motor, Usage usage) {
    setSparkMaxBusUsage(motor, usage, false);
  }
}

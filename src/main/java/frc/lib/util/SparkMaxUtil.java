package frc.lib.util;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
  public static void setSparkMaxBusUsage(SparkMax motor, Usage usage, boolean enableFollowing) {
    SparkMaxConfig config = new SparkMaxConfig();
        
    if (enableFollowing) {
      config.signals.outputCurrentPeriodMs(10);
    } else {
      config.signals.outputCurrentPeriodMs(500);
    }

    if (usage == Usage.kAll) {
      config.signals.warningsPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.analogPositionPeriodMs(50);
    } else if (usage == Usage.kPositionOnly) {
      config.signals.warningsPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(20);
      config.signals.analogPositionPeriodMs(500);
    } else if (usage == Usage.kVelocityOnly) {
      config.signals.warningsPeriodMs(20);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.analogPositionPeriodMs(500);
    } else if (usage == Usage.kMinimal) {
      config.signals.warningsPeriodMs(500);
      config.signals.primaryEncoderPositionPeriodMs(500);
      config.signals.analogPositionPeriodMs(500);
    }

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

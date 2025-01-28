package frc.lib.util;

import com.ctre.phoenix6.hardware.CANcoder;

/** Sets status frames for the CTRE CANCoder. */
public class CANCoderUtil {
  public enum CCUsage {
    kAll,
    kSensorDataOnly,
    kFaultsOnly,
    kMinimal
  }

  /**
   * This function allows reducing a CANCoder's CAN bus utilization by reducing the periodic status
   * frame period of nonessential frames from 10ms to 255ms.
   *
   * <p>See https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html#cancoder for a description
   * of the status frames.
   *
   * @param cancoder The CANCoder to adjust the status frames on.
   * @param usage The status frame feedback to enable. kAll is the default when a CANCoder
   *     isconstructed.
   */
  public static void setCANCoderBusUsage(CANcoder cancoder, CCUsage usage) {
    if (usage == CCUsage.kAll) {
      cancoder.getPosition().setUpdateFrequency(100);
      cancoder.getSupplyVoltage().setUpdateFrequency(100);
      cancoder.getFault_Undervoltage().setUpdateFrequency(100);
    } else if (usage == CCUsage.kSensorDataOnly) {
      cancoder.getPosition().setUpdateFrequency(100);
      cancoder.getSupplyVoltage().setUpdateFrequency(10);
      cancoder.getFault_Undervoltage().setUpdateFrequency(10);
    } else if (usage == CCUsage.kFaultsOnly) {
      cancoder.getPosition().setUpdateFrequency(10);
      cancoder.getSupplyVoltage().setUpdateFrequency(100);
      cancoder.getFault_Undervoltage().setUpdateFrequency(100);
    } else if (usage == CCUsage.kMinimal) {
      cancoder.getPosition().setUpdateFrequency(10);
      cancoder.getSupplyVoltage().setUpdateFrequency(10);
      cancoder.getFault_Undervoltage().setUpdateFrequency(10);
    }
  }
}

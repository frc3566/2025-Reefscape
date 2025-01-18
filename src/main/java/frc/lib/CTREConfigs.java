package frc.lib;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;

public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(Constants.Swerve.canCoderInvert)
        );

    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond; TODO: find deprecation replacement
  }
}

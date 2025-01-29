package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
// import com.ctre.phoenix.sensors.SensorTimeBase;


public final class CTREConfigs {
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANcoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.withMagnetSensor(
        new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(Constants.Swerve.canCoderInvert)
    );

    // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond; TODO: find deprecation replacement
  }
}

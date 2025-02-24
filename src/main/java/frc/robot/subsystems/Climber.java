package frc.robot.subsystems;


import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public SparkMax motor;
    public double speed = Constants.CLIMBER_SPEED; 

    public Climber() {
        motor = new SparkMax(9, MotorType.kBrushless); //TODO: add to constants
        motor.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void down() {
        motor.set(speed);
    }

    public void up() {
        motor.set(-speed*0.5);
    }

    public void stop() {
        motor.stopMotor();
    }

private SparkMaxConfig getMotorConfig(boolean isInverted) {
    // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(40)
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    return motorConfig;
    }
}
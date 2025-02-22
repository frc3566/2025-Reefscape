package frc.robot.subsystems;


import frc.robot.Constants;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Algae extends SubsystemBase {
    public SparkMax left, right;
    public double inSpeed = Constants.ALGAE_IN_SPEED;
    public double outSpeed = Constants.ALGAE_OUT_SPEED;

    public Algae() {
        left = new SparkMax(12, MotorType.kBrushless);
        right = new SparkMax(13, MotorType.kBrushless);
        left.configure(getMotorConfig(true), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void in() {
        left.set(inSpeed);
        right.set(inSpeed);
    }

    public void out() {
        left.set(-outSpeed); // negate speed to go out
        right.set(-outSpeed);
    }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    public void setVoltage(double volt) {
        left.setVoltage(volt);
        right.setVoltage(volt);
    }

private SparkMaxConfig getMotorConfig(boolean isInverted) {
    // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(20)
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    return motorConfig;
    }
}
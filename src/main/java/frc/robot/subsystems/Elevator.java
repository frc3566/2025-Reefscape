package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Elevator extends SubsystemBase {
    public SparkMax left, right;
    public double speed = Constants.ELEVATOR_SPEED;

    public Elevator() {
        left = new SparkMax(10, MotorType.kBrushless);
        right = new SparkMax(11, MotorType.kBrushless);
        left.configure(getMotorConfig(true), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        right.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up() {
        left.set(speed);
        right.set(speed);
    }

    public void down() {
        left.set(-speed);
        right.set(-speed);
    }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }

    public void setVolatge(double volt) {
        left.setVoltage(volt);
        right.setVoltage(volt);
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
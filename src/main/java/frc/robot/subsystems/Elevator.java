package frc.robot.subsystems;


import frc.robot.Constants;

import java.lang.constant.Constable;

import com.pathplanner.lib.events.TriggerEvent;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class Elevator extends SubsystemBase {
    public SparkMax motor, follower;
    public double speed = Constants.ELEVATOR_SPEED;

    public Elevator() {
        motor = new SparkMax(10, MotorType.kBrushless);
        follower = new SparkMax(11, MotorType.kBrushless);
        SparkMaxConfig motorConfig = getMotorConfig(false);
        SparkMaxConfig followerConfig = getMotorConfig(true);
        followerConfig.follow(motor, true);
        motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up() {
        motor.set(speed);
    }

    public void down() {
        motor.set(-speed);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
        System.out.println(getHeightMeters());
    }

    public void setVoltage(double volt) {
        motor.setVoltage(volt);
    }

    public double getHeightMeters() {
        return motor.getEncoder().getPosition() / Constants.ELEVATOR_GEAR_RATIO * 2 * Math.PI * Constants.ELEVATOR_DRUM_RADIUS; 
    }

    public double getEncoder() {
        return motor.getEncoder().getPosition();
    }

    public double getVelocityMPS() {
        return ((motor.getEncoder().getVelocity() / 60) / Constants.ELEVATOR_GEAR_RATIO) * (2 * Math.PI * Constants.ELEVATOR_DRUM_RADIUS);
    }
 
    private SparkMaxConfig getMotorConfig(boolean isInverted) {
        // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
            .smartCurrentLimit(20)
            .inverted(isInverted)
            .idleMode(IdleMode.kBrake);
        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.042864, 0, 0); //TODO: monitor 
        return motorConfig;
    }
}
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

    public void zeroPower(){
        left.set(0);
        right.set(0);
    }
    public void voltageDrive(double volt) {
        left.setVoltage(volt);
        right.setVoltage(volt);
    }

// private SysIdRoutine getSysIdRoutine(){
//     SysIdRoutine sysid = new SysIdRoutine(
//             new SysIdRoutine.Config(),
//             new SysIdRoutine.Mechanism(
//                 this::voltageDrive, 
//                 this::logMotors, 
//                 this
//             )
//         );
// }

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
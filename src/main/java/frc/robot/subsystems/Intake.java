package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final SparkMax intake = new SparkMax(15, MotorType.kBrushless);
    private final SparkMax pivot =  new SparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intake.getEncoder();
    private final RelativeEncoder rotationEncoder = intake.getEncoder();
    private final double pivotSpeed = 0.3;
    private final double intakeSpeed = 0.4;

    public Intake(){
        intake.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void up(){
        pivot.set(pivotSpeed);
    }

    public void down(){
        pivot.set(-pivotSpeed);
    }

    public void run(boolean in){
        intake.set(in ? intakeSpeed : -intakeSpeed);
    }

    public void stopIntake(){
        intake.stopMotor();
    }

    public void stopRotation(){
        pivot.stopMotor();
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

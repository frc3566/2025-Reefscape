package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private final SparkMax intake = new SparkMax(15, MotorType.kBrushless);
    private final SparkMax pivot =  new SparkMax(14, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intake.getEncoder();
    private final RelativeEncoder rotationEncoder = intake.getEncoder();
    private final double pivotSpeed = Constants.PIVOT_SPEED;
    private final double intakeSpeed = Constants.INTAKE_SPEED;

    public Intake(){
        intake.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        pivot.configure(getMotorConfig(false), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void runIntake(boolean in){
        intake.set(in ? intakeSpeed : -intakeSpeed);
        System.out.println("Intake degree:" + getPivotDegree());
    }

    public void set(double speed){
        pivot.set(speed);
    }

    public void set(boolean up){
        pivot.set(up ? pivotSpeed : -pivotSpeed);
        System.out.println("Intake degree:" + getPivotDegree());
    }

    public void stopIntake(){
        intake.stopMotor();
    }

    public void stopPivot(){
        pivot.stopMotor();
    }

    public double getPivotDegree(){
        return pivot.getEncoder().getPosition();
    }

    private SparkMaxConfig getMotorConfig(boolean isInverted) {
    // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .smartCurrentLimit(40)
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake);
    motorConfig.encoder
        .positionConversionFactor(360.0/(81.0/1.0));
    return motorConfig;
    }

    // private void getAngleMotorConfig() {

    //     SparkMaxUtil.setSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    //     SparkMaxConfig angleConfig = new SparkMaxConfig();
    //     angleConfig
    //         .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
    //         .inverted(Constants.Swerve.angleInvert)
    //         .idleMode(Constants.Swerve.angleNeutralMode)
    //         .voltageCompensation(Constants.Swerve.voltageComp);
    //     angleConfig.encoder
    //         .positionConversionFactor(Constants.Swerve.angleConversionFactor);
    //     angleConfig.closedLoop
    //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //TODO: remove if causing errors
    //         .pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

    //     angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
}

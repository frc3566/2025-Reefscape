package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.SparkMaxUtil;
import frc.lib.util.SparkMaxUtil.Usage;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private SparkMax angleMotor;
    private SparkMax driveMotor;

    private CANcoder angleEncoder;
    private RelativeEncoder integratedAngleEncoder, driveEncoder;

    private int driveMotorID;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Definition */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);

        /* Angle Motor Config */
        angleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();
        driveMotorID = moduleConstants.driveMotorID;

        /* Encoder Config */
        configEncoders();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which REV and CTRE are not

        // desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        desiredState.optimize(getState().angle);
        desiredState.cosineScale(getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public double getValue() {
        return getCanCoder().getDegrees() - angleOffset.getDegrees();
    }

    public double getValueWithUpdate() {
        return getCanCoderWithUpdate().getDegrees() - angleOffset.getDegrees();
    }

    public void resetToAbsolute() {
        integratedAngleEncoder.setPosition(integratedAngleEncoder.getPosition() % 360);
        integratedAngleEncoder.setPosition(getValueWithUpdate());
    }

    private void configEncoders() {
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);
        
        integratedAngleEncoder = angleMotor.getEncoder();
        resetToAbsolute();
    }

    private void configAngleMotor() {

        SparkMaxUtil.setSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig
            .smartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit)
            .inverted(Constants.Swerve.angleInvert)
            .idleMode(Constants.Swerve.angleNeutralMode)
            .voltageCompensation(Constants.Swerve.voltageComp);
        angleConfig.encoder
            .positionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //TODO: remove if causing errors
            .pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);

        angleMotor.configure(angleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // angleMotor.restoreFactoryDefaults();
        // SparkMaxUtil.setSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        // angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        // angleMotor.setInverted(Constants.Swerve.angleInvert);
        // angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        // integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        // angleController.setP(Constants.Swerve.angleKP);
        // angleController.setI(Constants.Swerve.angleKI);
        // angleController.setD(Constants.Swerve.angleKD);
        // angleController.setFF(Constants.Swerve.angleKFF);
        // angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        // angleMotor.burnFlash();
        // resetToAbsolute();
    }

    private void configDriveMotor() {
        SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .smartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit)
            .inverted(Constants.Swerve.driveInvert)
            .idleMode(Constants.Swerve.driveNeutralMode)
            .voltageCompensation(Constants.Swerve.voltageComp);
        driveConfig.encoder
            .velocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor)
            .positionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //TODO: remove if causing errors
            .pid(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);

        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // driveMotor.restoreFactoryDefaults();
        // SparkMaxUtil.setSparkMaxBusUsage(driveMotor, Usage.kAll);
        // driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        // driveMotor.setInverted(Constants.Swerve.driveInvert);
        // driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        // driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        // driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        // driveController.setP(Constants.Swerve.angleKP);
        // driveController.setI(Constants.Swerve.angleKI);
        // driveController.setD(Constants.Swerve.angleKD);
        // driveController.setFF(Constants.Swerve.angleKFF);
        // driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        // driveMotor.burnFlash();
        // driveEncoder.setPosition(0.0);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        } else {
            SparkClosedLoopController controller = angleMotor.getClosedLoopController();
            controller.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    ClosedLoopSlot.kSlot0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeedMetersPerSecond * 0.01))
                ? lastAngle
                : desiredState.angle;
        SparkClosedLoopController controller = angleMotor.getClosedLoopController();
        controller.setReference(angle.getDegrees(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Rotations));
    }

    public Rotation2d getCanCoderWithUpdate() {
        var posVal = angleEncoder.getAbsolutePosition().waitForUpdate(1);
        var val = Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Rotations));
        if (posVal.getStatus().isOK()) {
            val = Rotation2d.fromRotations(posVal.getValueAsDouble());
            System.out.println("Success in getting rotation! Angle for posval: "
                    + Rotation2d.fromRotations(posVal.getValueAsDouble()));
        } else {
            System.out.println("Failed to get accurate rotation! Error: " + posVal.getStatus());
        }
        return val;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                getAngle());
    }

    public int getDriveMotorID() {
        return driveMotorID;
    }

    public SparkMax getDriveMotor() {
        return driveMotor; 
    }

    public SparkMax getAngleMotor() {
        return angleMotor;
    }

    public double getAngleSpeed() {
        return angleEncoder.getVelocity().getValue().in(edu.wpi.first.units.Units.RotationsPerSecond);
    }
}

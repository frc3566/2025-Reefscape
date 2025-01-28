package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
// import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.SwerveVoltageRequest;

public class SysIdSwerve extends Swerve {
    /* Mutable Measures to keep track of */
    private final MutVoltage appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    private final MutDistance distance = MutableMeasure.mutable(Units.Meters.of(0));
    private final MutLinearVelocity velocity = MutableMeasure.mutable(Units.MetersPerSecond.of(0));
    private final MutAngle angle = MutableMeasure.mutable(Units.Radians.of(0));
    private final MutAngularVelocity angularVelocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));

    /* Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage. */
    private final SysIdRoutine.Config routineConfig = new SysIdRoutine.Config();

    private final SysIdRoutine.Mechanism routineMechanism = new SysIdRoutine.Mechanism(
        this::routineDriving,
        this::routineLogging,
        /* Tell SysId to make the generated commands require this subsystem, 
            suffix test state in WPILog with this subsystem's name ("drive") */
        this 
    );

    private final SwerveControlRequestParameters controlRequestParameters = new SwerveControlRequestParameters();

    /* set control request parameters */
    public SysIdSwerve() {
        super();

        controlRequestParameters.kinematics = Constants.Swerve.swerveKinematics;
        controlRequestParameters.currentPose = getPose();
        controlRequestParameters.swervePositions = Arrays.stream(getModulePositions())
            .map(modulePosition -> new Translation2d(modulePosition.distanceMeters, modulePosition.angle))
            .toArray(size -> new Translation2d[size]);

        /* TODO: set timestamp and updatePeriod values */
        controlRequestParameters.timestamp = 0;
        controlRequestParameters.updatePeriod = 0;
    }

    /* Tell SysId how to plumb the driving voltage to the motors. */
    private void routineDriving(Voltage volts) {
        appliedVoltage.mut_replace(volts);
        new SwerveVoltageRequest()
            .withVoltage(volts.in(Units.Volts))
            .apply(controlRequestParameters, mSwerveMods);
    }

    /* Tell SysId how to record a frame of data for each motor on the mechanism being characterized. */
    private void routineLogging(SysIdRoutineLog log) {
        /* Record a frame for every module. 
            Since each motor in a module shares an encoder, we consider the entire group to be one motor. */

        Arrays.asList(mSwerveMods).forEach(e -> {
            log.motor("Motor " + e.moduleNumber)
                .voltage(appliedVoltage)
                .linearPosition(distance.mut_replace(e.getPosition().distanceMeters, Units.Meters))
                .linearVelocity(velocity.mut_replace(e.getState().speedMetersPerSecond, Units.MetersPerSecond))
                .angularPosition(angle.mut_replace(e.getPosition().angle.getRadians(), Units.Radians))
                .angularVelocity(angularVelocity.mut_replace(e.getAngleSpeed(), Units.RotationsPerSecond));
        });
    }

    public SysIdRoutine getRoutine() {
        return new SysIdRoutine(
            routineConfig,
            routineMechanism
        );
    }

    public Command sysIdQuasistatic(Direction kforward) {
        return getRoutine().quasistatic(kforward);
    }

    public Command sysIdDynamic(Direction kforward) {
        return getRoutine().dynamic(kforward);
    }
}

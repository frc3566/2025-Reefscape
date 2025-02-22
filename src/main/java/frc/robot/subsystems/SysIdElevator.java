package frc.robot.subsystems;

import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.SwerveVoltageRequest;

public class SysIdElevator extends Elevator {
    /* Mutable Measures to keep track of */
    private final MutVoltage appliedVoltage = Units.Volts.mutable(0);
    private final MutDistance distance = Units.Meters.mutable(0);
    private final MutLinearVelocity velocity = Units.MetersPerSecond.mutable(0);
    private final MutAngle angle = Units.Radians.mutable(0);
    private final MutAngularVelocity angularVelocity = Units.RotationsPerSecond.mutable(0);

    /* Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage. */
    private final SysIdRoutine.Config routineConfig = new SysIdRoutine.Config();

    private final SysIdRoutine.Mechanism routineMechanism = new SysIdRoutine.Mechanism(
        this::routineSetVoltage,
        this::routineLogging,
        this
    );

    /* set control request parameters */
    public SysIdElevator() {
        super();

        // this.encoder.setDistancePerPulse(Constants.Shooter.kEncoderDistancePerPulse);
    }

    /* Tell SysId how to feed voltage to elevator motors */
    private void routineSetVoltage(Voltage volts) {
        appliedVoltage.mut_replace(volts);
        this.voltageDrive(
            volts.in(Units.Volts)
        );
    }

    /* Tell SysId how to record a frame of data for each motor on the mechanism being characterized. */
    private void routineLogging(SysIdRoutineLog log) {
        /* Record a frame for every module. 
            Since each motor in a module shares an encoder, we consider the entire group to be one motor. */

        // Record a frame for the shooter motor.
        log.motor("left_elevator")
            .voltage(appliedVoltage.mut_replace(
                this.getLeft().get() * RobotController.getBatteryVoltage(), 
                Units.Volts))
            .angularPosition(angle.mut_replace(this.getLeftAbsoluteEncoder().getPosition(), Units.Rotations))
            .angularVelocity(angularVelocity.mut_replace(this.getLeftAbsoluteEncoder().getVelocity(), Units.RotationsPerSecond));
        log.motor("right_elevator")
            .voltage(appliedVoltage.mut_replace(
                this.getRight().get() * RobotController.getBatteryVoltage(), 
                Units.Volts))
            .angularPosition(angle.mut_replace(this.getRightAbsoluteEncoder().getPosition(), Units.Rotations))
            .angularVelocity(angularVelocity.mut_replace(this.getRightAbsoluteEncoder().getVelocity(), Units.RotationsPerSecond));
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
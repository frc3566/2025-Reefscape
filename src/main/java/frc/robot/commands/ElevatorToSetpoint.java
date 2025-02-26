package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToSetpoint extends Command {
    private Elevator elevator;
    private ProfiledPIDController controller;
    private ElevatorFeedforward feedforward;
    
    public ElevatorToSetpoint(Elevator elevator){
        this.elevator = elevator;
        this.controller = new ProfiledPIDController(0.1, 0.0, 0.0, null);
        this.feedforward = new ElevatorFeedforward(0.22683, 0, 0.0675, 0.0053266);
    }

    @Override
    public void initialize() {

    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly
     * until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());

        double value = MathUtil.clamp(controller.calculate(elevator.getRotation(), 0.0), -0.5,
            0.5);
        elevator.setVoltage(value + feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
        controller.getSetpoint().velocity), -7, 7);
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes --
     * indicated by this method returning true --
     * the scheduler will call its {@link #end(boolean)} method.
     * </p>
     * <p>
     * Returning false will result in the command never ending automatically. It may
     * still be cancelled manually or
     * interrupted by another command. Hard coding this command to always return
     * true will result in the command executing
     * once and finishing immediately. It is recommended to use *
     * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such
     * an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called
     * when {@link #isFinished()} returns true -- or when it is
     * interrupted/canceled. This is where you may want to wrap
     * up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
    
    }

}


// m_Feedforward = new ElevatorFeedforward(0.22683, 0.0053266, 0.0675);
// m_controller = new controller(0.1, 0, 0);
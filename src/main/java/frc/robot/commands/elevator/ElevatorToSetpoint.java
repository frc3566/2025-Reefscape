package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
public class ElevatorToSetpoint extends Command {
    private final ProfiledPIDController controller;
    private final ElevatorFeedforward feedForward;
    private final Elevator elevator;
    private double setpoint; //HEIGHT
    
    public ElevatorToSetpoint(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        feedForward = new ElevatorFeedforward(0.22683, 0, 0.0678, 0.0053266); //TODO: do kG and test kS, kV, kA
        controller = new ProfiledPIDController(0.01, 0, 0.001, new Constraints(1, 0.5)); //TODO: change values, these are generic
        controller.setTolerance(0.02);
        addRequirements(elevator);
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double voltOut = MathUtil.clamp(controller.calculate(elevator.getHeightMeters(), setpoint) + feedForward.calculate(elevator.getVelocityMPS(), controller.getSetpoint().velocity), -12, 12);
        System.out.println("Height: " + elevator.getHeightMeters() + ", Voltage: " + voltOut + ", AtPoint: " + controller.atSetpoint());
        elevator.setVoltage(voltOut);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
    
}

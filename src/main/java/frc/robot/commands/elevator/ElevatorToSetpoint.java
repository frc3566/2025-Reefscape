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
        feedForward = new ElevatorFeedforward(0.22683, 0, 6.78, 0.53266); //TODO: do kG and test kS, kV, kA
        controller = new ProfiledPIDController(10, 0, 1, new Constraints(2, 1)); //TODO: change values, these are generic
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
        double voltOut = MathUtil.clamp(controller.calculate(elevator.getEncoder(), setpoint) + feedForward.calculate(elevator.getVelocityMPS(), controller.getSetpoint().velocity), -12, 12);
        System.out.println("Encoder: " + elevator.getEncoder() + ", Height: " + elevator.getHeightMeters() + ", Voltage: " + voltOut + ", AtPoint: " + controller.atSetpoint());
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

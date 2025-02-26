package frc.robot.commands.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
public class ElevatorToSetpoint extends Command {
    private final PIDController m_Controller;
    private final Elevator elevator;
    private double setpoint; //HEIGHT
    
    public ElevatorToSetpoint(Elevator elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;
        m_Controller = new PIDController(0.01, 0, 0.001); //TODO: change values, these are generic
        m_Controller.setTolerance(0.02);
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
        double value = MathUtil.clamp(m_Controller.calculate(elevator.getHeightMeters(), setpoint), 0.3, 1.6);
        System.out.println("Angle: " + elevator.getHeightMeters() + ", Value: " + value + ", AtPoint: " + m_Controller.atSetpoint());
        elevator.set(value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Controller.atSetpoint();
    }
    
}

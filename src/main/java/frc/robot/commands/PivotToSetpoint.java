package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
public class PivotToSetpoint extends Command {
    private final PIDController m_Controller;
    private final Intake m_Intake;
    
    public PivotToSetpoint(Intake m_Intake) {
        this.m_Intake = m_Intake;
        addRequirements(m_Intake);
        m_Controller = new PIDController(0.01, 0, 0.001);
        m_Controller.setTolerance(1.5);
        m_Controller.setSetpoint(45);



        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // double value = Math.clamp(m_Controller.calculate(m_Intake.getPivotDegrees(), 45), minval, maxval);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double value = MathUtil.clamp(m_Controller.calculate(m_Intake.getPivotDegree(), 45), -0.3, 0.3);
        System.out.println("Angle: " + m_Intake.getPivotDegree() + ", Value: " + value + ", AtPoint: " + m_Controller.atSetpoint());
        m_Intake.set(value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Controller.atSetpoint();
    }
    
}

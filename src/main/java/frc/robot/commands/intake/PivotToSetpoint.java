package frc.robot.commands.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
public class PivotToSetpoint extends Command {
    private final PIDController m_Controller;
    private final Intake m_Intake;
    private double setpoint; //ANGLE
    
    public PivotToSetpoint(Intake m_Intake, double setpoint) {
        this.m_Intake = m_Intake;
        this.setpoint = setpoint + Constants.PIVOT_OFFSET;
        addRequirements(m_Intake);
        m_Controller = new PIDController(0.05, 0, 0.001); //TODO: monitor values for accuracy
        m_Controller.setTolerance(1.5);



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
        double value = MathUtil.clamp(m_Controller.calculate(m_Intake.getPivotDegree(), setpoint), -0.3, 0.3);
        System.out.println("Angle: " + m_Intake.getPivotDegree() + ", Value: " + value + ", AtPoint: " + m_Controller.atSetpoint());
        m_Intake.set(value);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Intake.stopPivot();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Controller.atSetpoint();
    }
    
}

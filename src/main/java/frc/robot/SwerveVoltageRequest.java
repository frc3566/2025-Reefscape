package frc.robot;

import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

public class SwerveVoltageRequest {
    // private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0, false, 0, 0, false, false, false);
    // private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);
    
    private double m_targetVoltage = 0.0;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (SwerveModule module: modulesToApply) {
            // Command steer motor to zero, template code: { module.getSteerMotor().setControl(m_motionMagicControl); }
            if (module.getDriveMotorID() == 3 || module.getDriveMotorID() == 7){
                module.getAngleMotor().setVoltage(0);
                // Command drive motor to voltage, template code: { module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage)); }
                module.getDriveMotor().setVoltage(m_targetVoltage);
            }
            else{
                module.getAngleMotor().setVoltage(0);
                module.getDriveMotor().setInverted(true);
                module.getDriveMotor().setVoltage(m_targetVoltage);
            }
        }

        return StatusCode.OK;
    }

    /**
     * 
     * @param targetVoltage Voltage for all modules to target
     * @return 
     */
    public SwerveVoltageRequest withVoltage(double targetVoltage) {
        this.m_targetVoltage = targetVoltage;
        return this;
    }
}
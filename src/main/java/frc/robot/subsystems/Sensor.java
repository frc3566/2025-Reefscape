package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensor extends SubsystemBase {
    // Has to manually adjust detection distance, 5cm-80cm
    // About 0.5s lag? 
    private DigitalInput sensor; 

    public Sensor() {
        this.sensor = new DigitalInput(0);
    }

    public boolean isClose() {
        return !sensor.get(); //Return true if detects object in range
    }

}

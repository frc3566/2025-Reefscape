package frc.robot.commands;

/* 
 * Because wpilib commands' isFinished() method do not act as expected (they can return false after end() is called),
 * this interface provides a way to tell when a command is or is not running.
*/
public interface WithStatus {
    public abstract boolean isRunning();
}

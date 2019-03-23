package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.IO;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

public class Climb extends Command {

    public Climb() {
        requires(Robot.cs);
    }

    protected void initialize() {
        Robot.cs.setMotor(0);
    }
 
    protected void execute() {
        if (DriverStation.getInstance().getMatchTime() < 20)
            Robot.ls.climbMode = true;
     
        Robot.cs.setMotor(IO.climbWheels());
         
        if (IO.toggleClimb()) {
            Robot.cs.toggleSol();
        }
    }
 
    protected boolean isFinished() {
        return false;
    }
 
    protected void end() {
        Robot.cs.disable();
    }
 
    protected void interrupted() {
        Robot.cs.disable();
    }
}

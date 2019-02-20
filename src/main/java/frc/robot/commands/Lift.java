/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.IO;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

// The act of lifting the robot
public class Lift extends Command 
{
    // The max speed of the lift in (encoder-units/tick)
    private static final double LIFT_SPEED = .15; 

    public Lift() 
    {
        // Require the lift to operate
        requires(Robot.ls);
    }

    @Override
    protected void initialize() 
    {
        // Turn on the lift
        Robot.ls.enable();
    }

    // On each tick, move the height up by IO.lift() * LIFT_SPEED
    @Override
    protected void execute() 
    {
        double newHeight = Robot.ls.getSetpoint() + (LIFT_SPEED * IO.lift()); // The intended height
        SmartDashboard.putNumber("Lift", IO.lift());
        SmartDashboard.putNumber("Setpoint", Robot.ls.getSetpoint());

        if (!Robot.ls.isInit)
            Robot.ls.moveToHeight(newHeight); // Move to the intended height
    }

    // This command should always be running (There's no way off Paige's Wild Ride)
    @Override
    protected boolean isFinished() 
    {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() 
    {
        Robot.ls.disable();
    }
    
    // Disable the lift system
    @Override
    protected void interrupted() 
    {
        Robot.ls.disable();
    }
}
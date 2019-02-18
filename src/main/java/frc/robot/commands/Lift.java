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

public class Lift extends Command {
    private static final double LIFT_SPEED = .1; // The Max speed of the lift in Inches / Second

    public Lift() {
        requires(Robot.ls);
    }

    @Override
    protected void initialize() {
        Robot.ls.enable();
    }

    @Override
    protected void execute() 
    {
        Robot.ls.moveToHeight(Robot.ls.getSetpoint() + (LIFT_SPEED * IO.lift()));
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.ls.disable();
    }
    
    @Override
    protected void interrupted() {
        Robot.ls.disable();
    }
}
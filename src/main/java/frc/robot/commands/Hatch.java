/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.IO;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hatch extends Command {
    
    private static final double MIN_HEIGHT = 10;
    private static final double SAFE_ANGLE = 40;
    private static final double MAX_SPEED = .02;

    public Hatch() 
    {
        requires(Robot.hs);
    }

    
    protected void initialize() 
    {
        Robot.hs.enable();
    }

    
    protected void execute() {

        if (IO.toggleHatch())
            Robot.hs.toggleHatch();

        if (IO.toggleTilt())
            Robot.hs.toggleTilt();

        //  if (Math.abs(IO.hatchAxis()) > 0)
        Robot.hs.moveInc(-IO.hatchAxis() * MAX_SPEED);
        // else if (Robot.ls.getHeight() < MIN_HEIGHT && Robot.hs.getAngle() < SAFE_ANGLE)
        //     Robot.hs.moveTo(SAFE_ANGLE);
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() {
        Robot.hs.disable();
    }

    
    protected void interrupted() {
        Robot.hs.disable();
    }
}

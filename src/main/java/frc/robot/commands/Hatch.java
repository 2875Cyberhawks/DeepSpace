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

public class Hatch extends Command {
    
    public static final double CURRENT = 5;

    public Hatch() 
    {
        requires(Robot.hs);
    }
    
    protected void execute() 
    {
        if (IO.hatchIntake() && Robot.hs.getCurrent() < CURRENT)
            Robot.hs.setSpeed(1);
        else if (IO.hatchOutput())
            Robot.hs.setSpeed(-1);
        else
            Robot.hs.setSpeed(0);

        if (IO.thrust())
            Robot.hs.toggleSol();
    }

    protected boolean isFinished() 
    {
        return false;
    }
    
    protected void end() {
        Robot.hs.disable();
    }
    
    protected void interrupted() {
        Robot.hs.disable();
    }
}
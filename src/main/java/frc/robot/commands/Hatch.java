/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.HatchSystem;
import frc.robot.IO;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hatch extends Command {
    
    private static final double MIN_HEIGHT = 10;
    private static final double SAFE_ANGLE = 40;
    private static final double MAX_SPEED = .5;
    private static final double HATCH_VEL = .0075;

    public Hatch() 
    {
        requires(Robot.hs);
    }
    
    protected void initialize() 
    {
        ;;
    }

    
    protected void execute() 
    {
        if (IO.toggleHatch())
            Robot.hs.toggleHatch();

        if (IO.toggleTilt())
            Robot.hs.toggleTilt();

        if (IO.getCenterHatch())
        {
            if (!Robot.hs.limited)
                Robot.hs.setCent();

            Robot.hs.limited = !Robot.hs.limited;   
        }

        Robot.hs.moveInc(HATCH_VEL * HatchSystem.FULL_TURN * IO.hatchAxis());
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
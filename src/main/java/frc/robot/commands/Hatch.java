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
    public static final double CURRENT = 5;
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
        if (IO.hatchIntake() && Robot.hs.getCurrent()>5)
            Robot.hs.setSpeed(1);        
       if (IO.hatchOutput())
           Robot.hs.setSpeed(-1);

        if (IO.thrust())
            Robot.hs.thrust();   
        if (IO.getCenterHatch())
        {
            if (!Robot.hs.limited)
                Robot.hs.setCent();

            Robot.hs.limited = !Robot.hs.limited;   
        }
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
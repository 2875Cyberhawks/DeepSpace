/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.IO;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Hatch extends Command {
    
    public static final double CURRENT = 3;
    private static final double[] MOT_SPD = {.6, 1};
    private boolean backed = false;
    private boolean wasBacking = false;
    private static final double MAX_TIME = 1;
    private Timer time = new Timer();

    public Hatch() 
    {
        requires(Robot.hs);
    }
    
    protected void execute() 
    {
        if (IO.hatchIntake())
        {
            if (!wasBacking)
            {
                time.start();
                time.reset();
            }

            if (time.get() > MAX_TIME)
                time.stop();

            if (Robot.hs.getCurrent() > CURRENT)
                backed = true;
            
            Robot.hs.setSpeed(!backed ? -MOT_SPD[0] * time.get() : 0);
            
            wasBacking = true;
        }
        else if (IO.hatchOutput())
        {
            wasBacking = false;
            backed = false;
            Robot.hs.setSpeed(MOT_SPD[1]);
        }
        else
        {
            wasBacking = false;
            backed = false;
            Robot.hs.setSpeed(0);
        }

        if (IO.thrust())
            Robot.hs.toggleSol();
    }

    protected boolean isFinished() 
    {
        return false;
    }
    
    protected void end() 
    {
        Robot.hs.disable();
    }
    
    protected void interrupted() 
    {
        Robot.hs.disable();
    }
}
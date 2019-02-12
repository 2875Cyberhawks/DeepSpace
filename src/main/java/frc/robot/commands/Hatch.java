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
    
        public Hatch() {
            requires(Robot.hs);
    }

    
    protected void initialize() {
        Robot.hs.enable();
    }

    
    protected void execute() {

        if (IO.getYPressed())
            Robot.hs.toggleHatch();

        if (IO.getXPressed())
            Robot.hs.toggleTilt();

        Robot.hs.move(IO.hatchAxis());
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

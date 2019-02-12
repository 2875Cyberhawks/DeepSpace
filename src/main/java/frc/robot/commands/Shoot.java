/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.IO;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class Shoot extends Command {

    private static final double TIME = .25;
    private Timer timer = new Timer();

    public Shoot() {
        requires(Robot.bs);
    }


    protected void initialize() {
        Robot.bs.enable();
        timer.start();
    }

    
    protected void execute() {
        Robot.bs.setSpeed(IO.leftTrigger(), 0);
        Robot.bs.setSpeed(IO.leftTrigger(), 1);

        if(timer.get() > TIME)
            Robot.bs.setSpeed(IO.leftTrigger(), 2);
    }

    
    protected boolean isFinished() {
        return IO.leftTrigger() == 0;
    }


    protected void end() {
        timer.reset();
        Robot.bs.disable();
    }

    
    protected void interrupted() {
        timer.reset();
        Robot.bs.disable();
    }
}

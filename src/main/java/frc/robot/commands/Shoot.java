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
        requires(Robot.ss);
    }


    protected void initialize() {
        Robot.bs.enable();
        timer.start();
    }

    
    protected void execute() {
        if (IO.shootBall() > 0){
            timer.reset();
            timer.start();
            Robot.ss.setSpeed(-IO.shootBall(), 0);
            Robot.ss.setSpeed(IO.shootBall(), 1);

            if(timer.get() > TIME){
                Robot.ss.setSpeed(IO.shootBall(), 2);
                timer.reset();
            }
        }
        else if (IO.intakeBall() > 0){
            Robot.ss.setSpeed(IO.intakeBall(), 0);
            Robot.ss.setSpeed(-IO.intakeBall(), 1);
            Robot.ss.setSpeed(-IO.intakeBall(), 2);
        }
    }

    
    protected boolean isFinished() {
        return false;
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

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
import edu.wpi.first.wpilibj.command.Scheduler;

public class Ball extends Command {

    public Ball() {
        requires(Robot.bs);
    }


    protected void initialize() {
        Robot.bs.enable();
    }

    
    protected void execute() {
        Robot.bs.move(IO.ballAxis());

        if(IO.intakeBall() > 0){
            for (int i = 0; i < 3; i++)
                Robot.bs.setSpeed(IO.intakeBall(), i);
        }
        else if(IO.shootBall() > 0){
            Scheduler.getInstance().add(new Shoot());
        }
    }

    
    protected boolean isFinished() {
        return false;
    }

    
    protected void end() {
        Robot.bs.disable();
    }

    
    protected void interrupted() {
        Robot.bs.disable();
    }
}

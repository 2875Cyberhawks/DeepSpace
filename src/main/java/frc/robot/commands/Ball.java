package frc.robot.commands;

import frc.robot.IO;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Ball extends Command 
{
    private static final double MIN_HEIGHT = 10;
    private static final double SAFE_ANGLE = 20;
    private static final double MAX_BALL_SPEED = .0001;

    public Ball() 
    {
        requires(Robot.bs);
    }

    protected void initialize() 
    {
        Robot.bs.enable();
        // Robot.bs.moveTo(0);
    }
    
    protected void execute() 
    {
        if (IO.intakeBall() > 0)
        {
            Robot.bs.set(IO.intakeBall(), 0);
            Robot.bs.set(IO.intakeBall(), 1);
        }
        else if (IO.shootBall() > 0)
        {
            Robot.bs.set(-IO.shootBall(), 0);
            Robot.bs.set(-IO.shootBall(), 1);
        }
        else
        {
           Robot.bs.set(0, 0);
           Robot.bs.set(0, 1); 
        }
    }
    
    protected boolean isFinished() 
    {
        return false;
    }
    
    protected void end() 
    {
        Robot.bs.disable();
    }
 
    protected void interrupted() 
    {
        Robot.bs.disable();
    }
}// 
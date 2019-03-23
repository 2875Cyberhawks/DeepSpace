package frc.robot.commands;

import frc.robot.IO;
import frc.robot.Robot;
import frc.robot.subsystems.BallSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.command.Command;

public class Ball extends Command 
{
    private static final double MIN_HEIGHT = 10;
    private static final double SAFE_ANGLE = 20;
    private static final double MAX_BALL_SPEED = .0001;
    private static final double TURN_VEL = .005;

    public Ball() 
    {
        requires(Robot.bs);
    }

    protected void initialize() 
    {
        ;;
    }
    
    protected void execute() 
    {
        Robot.bs.moveInc(IO.ballAxis() * TURN_VEL * BallSystem.FULL_TURN);

        // if (IO.intakeBall() > 0)
        //     Robot.bs.shoot(IO.intakeBall());
        // else if (IO.shootBall() > 0)
        //     Robot.bs.shoot(-IO.shootBall());
        // else
        //    Robot.bs.shoot(0);

        double sht = IO.intakeBall() - IO.shootBall();
        Robot.bs.shoot(sht);

        SmartDashboard.putNumber("IO STATE", IO.intakeBall() - IO.shootBall());
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
}
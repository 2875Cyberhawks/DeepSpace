package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.subsystems.*;

import com.kauailabs.navx.frc.AHRS;

// Our robot
public class Robot extends TimedRobot
{
    public static AHRS gyro;

    public static DriveSystem ds;
    public static LiftSystem ls;
    public static BallSystem bs;
    public static HatchSystem hs;
    // public static ClimbSystem cs;

    private boolean startedAuto = false;

    // On robot startup:
    @Override
    public void robotInit()
    {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        ds = new DriveSystem();
        ls = new LiftSystem();

        hs = new HatchSystem();
        bs = new BallSystem();
        // cs = new ClimbSystem();

        System.out.println("boi.deploy() returned true\nboi.run()...");
    }

    @Override
    public void autonomousInit() 
    {
        startedAuto = true;
        gyro.reset();
        ls.init();
        bs.init();
    }

    @Override
    public void autonomousPeriodic() 
    {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() 
    {
        if (!startedAuto)
            autonomousInit();
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
    }
    
    public static double getAngle()
    {
        double gyAng = gyro.getAngle();

        while (gyAng < -180)
        {
            gyAng += 360;
        }
        while (gyAng > 180)
        {
            gyAng -= 360;
        }

        return gyAng;
    }
}
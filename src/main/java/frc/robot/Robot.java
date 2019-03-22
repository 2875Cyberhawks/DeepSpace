package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.subsystems.*;
import frc.robot.util.CamAlt;

import com.kauailabs.navx.frc.AHRS;

// Our robot
public class Robot extends TimedRobot
{
    public static AHRS gyro;
    public static CamAlt cam;

    public static DriveSystem ds;
    public static LiftSystem ls;
    public static BallSystem bs;
    public static HatchSystem hs;
    // public static ClimbSystem cs;

    private boolean startedAuto = false;

    SendableChooser<Boolean> startChooser;
    
    // On robot startup:
    @Override
    public void robotInit()
    {
        gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        cam = new CamAlt(0, 1);

        ds = new DriveSystem();
        ls = new LiftSystem();

        hs = new HatchSystem();
        bs = new BallSystem();
        // cs = new ClimbSystem();

        System.out.println("Robot Starting");

        startChooser = new SendableChooser<Boolean>();
        startChooser.setDefaultOption("Starting hatch", false);
        startChooser.addOption("Starting ball", true);
        SmartDashboard.putData(startChooser);
    }

    public void commonInit()
    {
        bs.init();
        hs.init();
    }

    @Override
    public void autonomousInit() 
    {
        ds.shootStart = (boolean) startChooser.getSelected();
        startedAuto = true;
        gyro.reset();
        cam.init();
        ls.init();
        commonInit();
    }

    @Override
    public void autonomousPeriodic() 
    {
        teleopPeriodic();
    }

    @Override
    public void teleopInit() 
    {
        commonInit();
        if (!startedAuto)
            autonomousInit();
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        if (IO.switchCamera())
            cam.toggle();

        Scheduler.getInstance().run();
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
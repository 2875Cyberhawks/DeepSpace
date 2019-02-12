package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LiftSystem;
import frc.robot.util.Vector;
import frc.robot.subsystems.HatchSystem;
import frc.robot.subsystems.BallSystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

// Our robot
public class Robot extends TimedRobot
{
    public static Joystick joy;

    public static XboxController xbox;

    public static AHRS gyro;

    public static DriveSystem ds;

    public static LiftSystem ls;

    public static HatchSystem hs;

    public static BallSystem bs;

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

        System.out.println("boi.deploy() returned true\nboi.run()...");
    }

    @Override
    public void autonomousInit() {
        gyro.reset();
    }

    @Override
    public void autonomousPeriodic() 
    {
        Vector[][] rots = Drive.turnToAngle(gyro.getAngle(), 90, 1.8, 0);
        ds.setVects(rots);
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
    }
}

package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveSystem;
import frc.robot.util.Vector;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

// Our robot
public class Robot extends TimedRobot
{
    public static Joystick joy;

    public static AHRS gyro;

    public static DriveSystem ds;

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

        System.out.println("SwerveDrive 4.0.0 is up!");
    }

    @Override
    public void autonomousPeriodic() 
    {
        Vector[][] rots = Drive.turnToAngle(gyro.getAngle(), 90);
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
    }
}

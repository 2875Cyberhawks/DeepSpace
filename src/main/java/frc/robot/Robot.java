package frc.robot;

import frc.robot.subsystems.DriveSystem;

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
    public void autonomousInit() 
    {
        super.autonomousInit();
    }

    // On teleop begin:
    @Override
    public void teleopInit()
    {
        ds.lastAngle = gyro.getAngle(); // Default angle is zero
        gyro.reset(); // Reset gyro
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
    }
}

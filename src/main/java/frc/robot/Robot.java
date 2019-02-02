package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;

// Our robot
public class Robot extends TimedRobot
{
    public static Joystick joy;

    public static AHRS gyro;

    public static DriveSystem ds;

    public static final double LENGTH = 22.5; // Length between the wheels in inches
    public static final double WIDTH = 20; // Width between the wheels in inches

    // On robot startup:
    @Override
    public void robotInit()
    {
        joy = new Joystick(0);

        gyro = new AHRS(SPI.Port.kMXP);
        
        gyro.reset();

        ds = new DriveSystem();

        System.out.println("SwerveDrive 3.0.0 is up!");
    }

    // On teleop begin:
    @Override
    public void teleopInit()
    {
        ds.turnMeant = false;
        ds.lastAngle = 0;
        gyro.reset();
        gyro.enableLogging(true);
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
    }
}

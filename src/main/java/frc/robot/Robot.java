package frc.robot;

// import frc.robot.commands.Drive;
// import frc.robot.subsystems.DriveSystem;
// import frc.robot.subsystems.LiftSystem;
import frc.robot.util.Vector;

// import frc.robot.subsystems.BallSystem;
// import frc.robot.subsystems.ClimbSystem;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;
import frc.robot.subsystems.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/* TODO:
    - Remove Constants from IO
    - Comments
*/

// Our robot
public class Robot extends TimedRobot
{
    public static AHRS gyro;

    // public static DriveSystem ds;
    public static LiftSystem ls;
    // public static BallSystem bs;

    // public static VisionSink vis = new VisionSink()
    // public static UsbCamera[] cams = new UsbCamera[2];

    public static HatchSystem hs;
    // public static ClimbSystem cs;

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

        // for (int i = 0; i < 2; i++)
        //     cams[i] = CameraServer.getInstance().startAutomaticCapture(i);
        
        // ds = new DriveSystem();
        ls = new LiftSystem();

        hs = new HatchSystem();
        // bs = new BallSystem();
        // cs = new ClimbSystem();

        System.out.println("boi.deploy() returned true\nboi.run()...");
    }

    @Override
    public void autonomousInit() 
    {
        gyro.reset();
    }

    @Override
    public void autonomousPeriodic() 
    {
        // Vector[][] rots = Drive.turnToAngle(gyro.getAngle(), 90, 1.8, 0);
        // ds.setVects(rots);

        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    @Override
    public void teleopInit() 
    {
        ls.init();
    }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    @Override
    public void testInit() 
    {
    }

    @Override
    public void testPeriodic() 
    {
        
    }
}
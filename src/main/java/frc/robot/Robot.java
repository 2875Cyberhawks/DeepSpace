package frc.robot;

import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveSystem;
import frc.robot.util.Vector;
import frc.robot.util.CentPot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    public void autonomousInit() 
    {
        gyro.reset();
    }

    @Override
    public void autonomousPeriodic() 
    {
        Vector[][] rots = Drive.turnToAngle(gyro.getAngle(), 90, 1.8, 0);
        ds.setVects(rots);
    }

    // @Override
    // public void teleopInit() {
    //     gyro.reset();
    // }

    // On each step during periodic:
    @Override
    public void teleopPeriodic()
    {
        Scheduler.getInstance().run(); // Run all the commands as specified by the scheduler
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    }

    private CentPot tst;

    @Override
    public void testInit()
    {
        ds.free();
        tst = new CentPot(2, 360, 0, -0.7130228173596729);

        /*int[][] ENC_PORTS = { { 1, 0 }, { 3, 2 } }; // Ports of the encoders

        pots = new AnalogPotentiometer[2][2];

        double[][] avg = new double[2][2];

        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++)
                pots[i][j] = new AnalogPotentiometer(ENC_PORTS[i][j], 2, -1);
        }

        for (int t = 0; t < 500; t++){
            for (int i = 0; i < 2; i++){
                for (int j = 0; j < 2; j++){
                    avg[i][j] += pots[i][j].get();
                }
            }
        }

        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++){
                avg[i][j] /= 500;
                System.out.println(i + " " + j + " - " + avg[i][j]);
            }
        }*/
    
    }           

    // Testing:
    @Override
    public void testPeriodic()
    {
        System.out.print("Val: ");
        System.out.println("Out: " + tst.get());
    }
}

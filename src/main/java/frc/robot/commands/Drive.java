/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.util.Vector;
import frc.robot.IO;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.DriveSystem;

// The act of driving the robot
public class Drive extends Command
{
    private static final double P_DEF = 1.9, D_DEF = .2; // The P and D constants of the control loop
    private static final double MAX_CORRECT = .75; // The maximum amount the correction should correct to
    private static final double MIN_VOLTAGE = 6.5; // The mimum voltage at which to perform corrections

    // Creates the drive object
    public Drive()
    {
        requires(Robot.ds); // Requires the drivetrain to be freed up
    }

    // Given a target angle and the current gyro position, return the rotational vectors needed to achieve that angle
    public static Vector[][] turnToAngle(double gyAng, double targAng)
    {
        return turnToAngle(gyAng, targAng, P_DEF, D_DEF);
    }

    public static Vector[][] turnToAngle(double gyAng, double targAng, double P, double D)
    {
        Vector[][] rots = new Vector[2][2];

        if (RobotController.getBatteryVoltage() <= MIN_VOLTAGE) // Check for underflow
            rots = DriveSystem.getRots(0);
        else // Correct otherwise
            {
            double error = (targAng - gyAng);
            if (error < -180)
                error += 360;
            else if (error > 180)
                error -= 360;
            error /= 180;

            double tV = (-D * Robot.gyro.getRate()) + (P * error);
            rots = DriveSystem.getRots(Math.abs(tV) > MAX_CORRECT ? Math.abs(tV) / tV : tV);
        }

        return rots;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Robot.ds.enable();
        Robot.ds.lastAngle = Robot.gyro.getAngle();

        Robot.ds.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        if (IO.getReset()) // If reset putton pushed
        {
            Robot.gyro.reset(); // Reset the gyro
            Robot.ds.lastAngle = 0; // The last intended angle is 0
        }

        double gyAng = Robot.getAngle(); // Angle of the gyroscope

        Vector trans = IO.trans();
        trans.setAngle(-gyAng + trans.getAngle());
        trans.circlify();

        double rMag = IO.z();

        Vector[][] rots; // The magnitude of the rotational velocity vector

        if (rMag == 0) // If magnitude is below the deadzone
            rots = turnToAngle(gyAng, Robot.ds.lastAngle);
        else
        {
            Robot.ds.lastAngle = gyAng;
            rots = DriveSystem.getRots(rMag);
        }

        Vector[][] totals = new Vector[2][2]; // The total sum of the two vectors for each wheel
        double max = 0; // The largest possible sum of the two vectors
        
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                totals[i][j] = Vector.add(rots[i][j], trans); // Add the rotational and translational vector
                
                if (totals[i][j].getMag() > max) // If the total is greater then the max:
                    max = totals[i][j].getMag(); // It is the new max
            }
        
        if (max > 1)
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    totals[i][j].scale(1 / max);

        Robot.ds.setVects(totals);
    }
 
    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() 
    {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() 
    {
        Robot.ds.disable();
    }

    // Called when another command which requires one or more of the same subsystems is scheduled to run
    @Override
    protected void interrupted() 
    {
        Robot.ds.disable();
    }
}
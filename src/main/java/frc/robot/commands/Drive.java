/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Vector;
import frc.robot.IO;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.DriveSystem;

// The act of driving the robot
public class Drive extends Command
{
    public static final double IN_DEAD = .2; // The input (from 0-1) required to actually make the robot respond
    private static final double P = 1.9, D = .2; // The P and D constants of the control loop
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

            double dV = -D * Robot.gyro.getRate();
            SmartDashboard.putNumber("D", dV);

            double pV = error * P;
            SmartDashboard.putNumber("P", pV);

            double tV = dV + pV;
            rots = DriveSystem.getRots(Math.abs(tV) > MAX_CORRECT ? Math.abs(tV) / tV : tV);
            SmartDashboard.putNumber("T",tV);
        }

        return rots;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Robot.ds.enable();
        Robot.ds.lastAngle = Robot.gyro.getAngle();

        // For each position
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Robot.ds.setTurn(i, j, 0); // Set turning to 0
                Robot.ds.setPower(i, j, 0); // Set power to 0
            }
        }
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

        double gyAng = Robot.gyro.getAngle(); // Angle of the gyroscope

        // If the gyAng > 180 or gyAng < -180, move it back onto the correct domain
        while (gyAng < -180)
        {
            gyAng += 360;
        }
        while (gyAng > 180)
        {
            gyAng -= 360;
        }

        Vector trans = IO.trans();
        double transMag = IO.trans().getMag(); // The magnitude of the translational velocity vector

        double transTheta = -gyAng + IO.trans().getAngle(); // The angle of the translational velocity vector
        trans.setPol(transMag, transTheta); // Apply the magnitude and angle to the translational velocity vector
        SmartDashboard.putString("trans initial", trans.toStringPol());

        if (trans.getMag() < IN_DEAD) // If less than the deadzone, set to zero
            trans.setPol(0, 0);

        trans.circlify();

        double rMag = IO.z();

        Vector[][] rots = DriveSystem.getRots(rMag); // The magnitude of the rotational velocity vector

        if (Math.abs(rMag) < IN_DEAD) // If magnitude is below the deadzone
            rots = turnToAngle(gyAng, Robot.ds.lastAngle);
        else
            Robot.ds.lastAngle = gyAng;

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
                SmartDashboard.putString("rot"+i+j, rots[i][j].toStringPol());

        Vector[][] totals = new Vector[2][2]; // The total sum of the two vectors for each wheel
        double max = 0; // The largest possible sum of the two vectors
        
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                totals[i][j] = Vector.add(rots[i][j], trans); // Add the rotational and translational vector
                SmartDashboard.putString("rawTot"+i+j, totals[i][j].toStringPol());
                if (totals[i][j].getMag() > max) // If the total is greater then the max:
                    max = totals[i][j].getMag(); // It is the new max
            }
        
        if (max > 1)
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                    totals[i][j].scale(1 / max);

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                Vector total = totals[i][j]; // The total-th total is that at i, j
                
                if (total.getMag() > IN_DEAD) // If the robot is not intended to be stationary
                {
                    double err = total.getAngle() - Robot.ds.encoders[i][j].get();

                    if (Math.abs(err) > 90)
                        total.negate();

                    SmartDashboard.putString("tot"+i+j, totals[i][j].toStringPol());
                    Robot.ds.setTurn(i, j, total.getAngle()); // Set the position to the angle of the vector
                    
                    Robot.ds.setPower(i, j, Math.cos(Math.toRadians(err)) * total.getMag()); // Set the power of each wheel to the magnitude of the vector
                }
                else
                    Robot.ds.setPower(i, j, 0);
            }
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

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() 
    {
        Robot.ds.disable();
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Vector;
import frc.robot.IO;

// The act of driving the robot
public class Drive extends Command
{
    private static final double TURN_ANGLE = Math.toDegrees(Math.atan2(Robot.WIDTH, Robot.LENGTH)); // The reference angle each wheel's turning angle is based on

    private static final double[][] RELATIVE_ANGLES = {{TURN_ANGLE, 90 + TURN_ANGLE}, // The angle of each wheel when turning
        {-TURN_ANGLE, TURN_ANGLE - 180}};

    public static final double IN_DEAD = .2; // The input (from 0-1) required to actually make the robot respond

    private static final double TURN_ERROR = 5.5; // How much the robot should respond to error
    private static final double ERROR_NEG = .02; // How small the turn error must be to be negligable ([-1,1]/sec)
    private static final double MAX_TURN_SPEED = .75; // The maximum turning speed
    private static final double MAX_TRANS_SPEED = .95; // The maximum translational speed
    private static final double MAX_SLOWDOWN_TIME = .07; // The maximum amount of time allocated to slowing the robot down after turn

    private static final Timer time = new Timer();
    // Creates the drive object
    public Drive()
    {
        requires(Robot.ds); // Requires the drivetrain to be freed up
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Robot.ds.enable();
        time.stop();
        time.reset();

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

        double transMag = IO.transMag(); // The magnitude of the translational velocity vector

        double transTheta = -gyAng + IO.transDirection(); // The angle of the translational velocity vector

        Vector trans = new Vector(); // The translational velocity vector
        trans.setPol(transMag, transTheta); // Apply the magnitude and angle to the translational velocity vector

        if (trans.getMag() < IN_DEAD) // If less than the deadzone, set to zero
            trans.setPol(0, 0);
        
        trans.circlify(); // This may not be needed?

        double rotMag = IO.z(); // The magnitude of the rotational velocity vector

        Vector[][] totals = {{null, null}, {null, null}}; // The total sum of the two vectors for each wheel
        double max = 0; // The largest possible sum of the two vectors

        Robot.ds.grabVal();

        SmartDashboard.putBoolean("isTurning", Robot.ds.isTurning());
        SmartDashboard.putBoolean("turnMeant", Robot.ds.turnMeant);
        SmartDashboard.putBoolean("tryTurning", rotMag > IN_DEAD);
        SmartDashboard.putNumber("timer", time.get());

        if (time.get() > MAX_SLOWDOWN_TIME)
        {
            time.stop();
            time.reset();
            
            Robot.ds.turnMeant = false;
            Robot.ds.lastAngle = Robot.gyro.getAngle();
        }

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                Vector rot = new Vector(); // The rotational velocity vector
                rot.setPol(rotMag, RELATIVE_ANGLES[i][j]); // Apply the magnitude and the relative angle to the 'rot' variable
                // Rot is already circle-ified, no need to change

                if (!Robot.ds.isTurning()) // If the robot isn't turning, then any further turns aren't meant
                    Robot.ds.turnMeant = false;

                if (rot.getMag() < IN_DEAD) // If magnitude is below the deadzone
                {
                    rot.setCart(0, 0); // Set the rotational velocity to zero
                    
                    // If the robot is turning 
                    if (Robot.ds.isTurning())
                    {
                        // If the turn 'is meant'
                        if (Robot.ds.turnMeant)
                        {
                            // Start the timer
                            time.start();
                        }
                        else // If the turn isn't meant
                        {
                            // Correct for error
                            double error = (Robot.ds.lastAngle - Robot.gyro.getAngle()) / 360;
                            SmartDashboard.putNumber("error", error);
                            rot.setPol(error * TURN_ERROR, RELATIVE_ANGLES[i][j]);
                            Robot.ds.rmSpdCache();
                        }
                    }
                }
                else // If magnitude's greater than deadzone
                {
                    time.stop(); 
                    time.reset();
                    
                    Robot.ds.rmSpdCache();
                    Robot.ds.turnMeant = true; // The turning is meant
                    Robot.ds.lastAngle = Robot.gyro.getAngle(); // The current angle is the right one
                }

                totals[i][j] = Vector.add(rot, trans); // Add the rotational and translational vector
                if (totals[i][j].getMag() > max) // If the total is greater then the max:
                    max = totals[i][j].getMag(); // It is the new max
            }

        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++)
            {
                Vector total = totals[i][j]; // The total-th total is that at i, j

                if (total.getMag() > IN_DEAD) // If the robot is not intended to be stationary
                {   
                    double err = total.getAngle() - Robot.ds.encoders[i][j].get();
                    
                    SmartDashboard.putNumber("Error"+i+j, err);

                    if (Math.abs(err) > 90)
                        total.negate();

                    SmartDashboard.putString("Total Vector", total.toString());

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
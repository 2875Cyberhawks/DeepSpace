/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.util.Vector;

// The act of driving the robot
public class Drive extends Command {

    private static final double TURN_ANGLE = Math.toDegrees(Math.atan2(Robot.WIDTH, Robot.LENGTH)); // The reference angle each wheel's turning angle is based on

    private static final double[][] RELATIVE_ANGLES = {{TURN_ANGLE, 90 + TURN_ANGLE}, // The angle of each wheel when turning
        {-TURN_ANGLE, TURN_ANGLE - 180}};

    public static final double TURN_SPEED = .75; // The maximum speed (0-1) to go when turning

    public static final double IN_DEAD = .15; // The input (from 0-1) required to actually make the robot respond

    public static final double TURN_ERR_DEAD = 5/360; // The error (from 0-1) required for the robot to automatically adjust the angle

    public static final double MIN_ANG_ROT = 35; // How fast the robot has to be rotating (in deg/sec) for the rotation to be considered intentional

    private static final double TURN_ERROR = 8; // How much the robot should respond to error

    private double lastAngle = 0; // The last angle considered 'intentional'

    private static final double MAX_ANG_ROT = .05; // The speed (deg/sec) at which anything below is considered intentionally stopped

    private static final double STAT_DEAD = .1; // The maximum total speed at which the robot is considered stationary

    // Creates the drive object
    public Drive()
    {
        requires(Robot.ds); // Requires the drivetrain to be freed up
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
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
    protected void execute() {
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

        double transMag = Robot.joy.getMagnitude(); // The magnitude of the translational velocity vector

        double transTheta = -gyAng + Robot.joy.getDirectionDegrees(); // The angle of the translational velocity vector

        Vector trans = new Vector(); // The translational velocity vector
        trans.setPol(transMag, transTheta); // Apply the magnitude and angle to the translational velocity vector

        if (trans.getMag() < IN_DEAD) // If less than the deadzone, set to zero
            trans.setPol(0, 0);

        double rotMag = Robot.joy.getZ() * TURN_SPEED; // The magnitude of the rotational velocity vector


        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Vector rot = new Vector(); // The rotational velocity vector
                rot.setPol(rotMag, RELATIVE_ANGLES[i][j]); // Apply the magnitude and the relative angle to the 'rot' variable
                
                // SmartDashboard.putNumber("Angle", Robot.gyro.getAngle());
                // SmartDashboard.putNumber("Last:", lastAngle);

                if (rot.getMag() < IN_DEAD) // If magnitude is below the deadzone
                {
                    rot.setCart(0, 0); // Set the rotational velocity to zero

                    if ((Math.abs(Robot.gyro.getRate()) < MIN_ANG_ROT) && (Math.abs(Robot.gyro.getRate()) > MAX_ANG_ROT)) // If the robot is moving slower than the Minimum Angle Rotation Speed and faster than the maximum
                    {
                        double error =  (lastAngle - Robot.gyro.getAngle()) / 360; // Error between the intended angle current angle on domain [0,1]

                        // SmartDashboard.putBoolean("correcting", true);
                        // if (Math.abs(error) > TURN_ERR_DEAD) // If the error is large enough
                        rot.setPol(TURN_ERROR * error, RELATIVE_ANGLES[i][j]); // Turn the robot with a magnitude directly related to the error
                    }
                    else
                        lastAngle = Robot.gyro.getAngle(); // If the robot is moving quickly or is stopped, set the intended angle to the current angle
                }
                else
                    lastAngle = Robot.gyro.getAngle();
            
                Vector total = Vector.add(rot, trans); // Add the rotational and translational vector

                total.scale(1 / (Math.sqrt(1 + (TURN_SPEED * TURN_SPEED)))); // Normalize it to the maximum speed
                double err = Robot.ds.getError(i, j);


                if (total.getMag() > STAT_DEAD) // If the robot is not intended to be stationary
                {
                    Robot.ds.setTurn(i, j, total.getAngle()); // Set the position to the angle of the vector

                    Robot.ds.setPower(i, j, Math.cos(Math.toRadians(err)) * total.getMag()); // Set the power of each wheel to the magnitude of the vector
                }
                else
                    Robot.ds.setPower(i, j, 0);
            }
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
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import edu.wpi.first.wpilibj.Encoder;

// The physical lift itself
public class LiftSystem extends PIDSubsystem {

    // The P, I and D constants
    private static final double P = .65;
    private static final double I = 0;
    private static final double D = 0;

    // The maximum speed
    private static final double MAX_SPD = .7;

    // The maximum error
    private static final double MAX_ERROR = 10;

    // The ports for the Spark motorcontrollers
    private static final int[] MOTOR_PORTS = {9, 8};

    // The ports for the encoders
    private static final int[] ENC_PORTS = {2, 3};

    // The encoder for tracking the height of the lift
    private Encoder encoder;
   
    // The motor(s)? that move the lift
    private SpeedControllerGroup motors;

    // The minimum height the lift should reach
    private static final double MIN_HEIGHT = 1; // Needs to be increased

    // The maximum height the lift should reach
    private static final double MAX_HEIGHT = 50; // Actually around 27

    // Constant of proportionality between the number of encoder pulses and inches
    private static final double DISTANCE_PER_PULSE = (12.0 / 577.0);
        
    public LiftSystem() 
    {
        // Tell the superclass the PID values
        super(P, I, D);

        // The input should only be on this range
        setInputRange(MIN_HEIGHT, MAX_HEIGHT);

        // The output should only be on this range 
        setOutputRange(-1, 1);

        encoder = new Encoder(ENC_PORTS[0], ENC_PORTS[1]); // Create the object
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE); // Set distance per pulse 

        // Create the SpeedControllerGroup
        motors = new SpeedControllerGroup(new Spark(MOTOR_PORTS[0]),
                                          new Spark(MOTOR_PORTS[1]));
        
        setSetpoint(MIN_HEIGHT);
    }

    @Override
    public void initDefaultCommand() 
    {
        // Run the lift command when nothing else requires the lift
        setDefaultCommand(new Lift());
    }

    @Override
    protected double returnPIDInput() 
    {
        // The input to the PID loop is the current height
        SmartDashboard.putNumber("Real Height", getHeight());
        return getHeight();
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        if (super.getPIDController().getError() > MAX_ERROR)
        {
            setSetpoint(getHeight());
            motors.set(0);
            return;
        }

        // The output of the PID loop should be the motor's speed
        if (output > MAX_SPD)
            motors.set(MAX_SPD);
        else if (output < -MAX_SPD)
            motors.set(-MAX_SPD);
        else
            motors.set(output);
    }

    // Move to a given height (double)
    public void moveToHeight(double height)
    {
        SmartDashboard.putNumber("targHeight", height);

        if (height > MAX_HEIGHT) // If the height is greater than the max:
        {
            // Move it to the max height
            SmartDashboard.putNumber("height", MAX_HEIGHT);
            setSetpoint(MAX_HEIGHT);
        }
        else if (height < MIN_HEIGHT) // otherwise, if the height is less than the min
        {
            // Move it to the minimum
            SmartDashboard.putNumber("height", MIN_HEIGHT);
            setSetpoint(MIN_HEIGHT);
        }
        else // otherwise, move it to the given height
        {    
            SmartDashboard.putNumber("height", height);
            setSetpoint(height);
        }
    }

    // Getting the height is the current distance of the encoder
    public double getHeight()
    {
        return encoder.getDistance();
    }

    // Disable the motors
    public void disable()
    {
        super.disable();
        motors.set(0);
    }

    // Free all of the HAL objects
    public void free()
    {
        super.free();
        motors.free();
        encoder.free();
    }

    // Reset the encoder
    public void reset()
    {
        encoder.reset();
    }
}
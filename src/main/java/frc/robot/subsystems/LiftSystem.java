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


public class LiftSystem extends PIDSubsystem {

    private static final double P = .7;
    private static final double I = 0.1;
    private static final double D = 0;

    private static final int[] MOTOR_PORTS = {9, 8};

    private static final int[] ENC_PORTS = {2, 3};

    private Encoder encoder;
    
    private SpeedControllerGroup motors;

    private static final double MIN_HEIGHT = 3;
    private static final double MAX_HEIGHT = 25;
    private static final double DISTANCE_PER_PULSE = (12.0 / 577.0);
        
    public LiftSystem() 
    {
        super(P, I, D);
        setInputRange(MIN_HEIGHT, MAX_HEIGHT);
        setOutputRange(-1, 1);

        encoder = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        encoder.reset();

        Spark[] spks = {new Spark(MOTOR_PORTS[0]), new Spark(MOTOR_PORTS[1])}; 
        motors = new SpeedControllerGroup(spks[0], spks[1]);
    }

    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Lift());
    }

    @Override
    protected double returnPIDInput() 
    {
        return getHeight();
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        motors.set(output);
    }

    public void moveToHeight(double height)
    {
        SmartDashboard.putNumber("targHeight", height);

        if (height > MAX_HEIGHT)
        {
            SmartDashboard.putNumber("height", MAX_HEIGHT);
            setSetpoint(MAX_HEIGHT);
        }
        else if (height < MIN_HEIGHT)
        {
            SmartDashboard.putNumber("height", MIN_HEIGHT);
            setSetpoint(MIN_HEIGHT);
        }
        else
        {    
            SmartDashboard.putNumber("height", height);
            setSetpoint(height);
        }
    }

    public double getHeight()
    {
        return encoder.getDistance();
    }

    public void disable()
    {
        super.disable();
        motors.set(0);
    }

    public void free()
    {
        motors.free();
        super.free();
        encoder.free();
    }
}
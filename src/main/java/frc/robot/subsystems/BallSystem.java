/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Ball;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallSystem extends PIDSubsystem {

    private static final double P = .5;
    private static final double I = 0;
    private static final double D = 0;

    private static final int[] DEVICE_NUMS = {1, 12, 10}; // Turn, Lower, Upper

    public static final double MIN = 3470, ZERO = 6750, MAX = 7000; // Forward, Straight, Back
    private TalonSRX rotTal = new TalonSRX(DEVICE_NUMS[0]);

    private static final double MAX_VOLTAGE = .4;

    private static final double MAX_TURN_SPEED = 100;

    private Talon[] motors = {new Talon(DEVICE_NUMS[1]), new Talon(DEVICE_NUMS[2])};
    
    public BallSystem() 
    {
        super(P, I, D);

        setOutputRange(-1, 1);

        for (int i = 0; i < 2; i++)
            motors[i].set(0);
    }

    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Ball());    
    }

    @Override
    protected double returnPIDInput() 
    {
        return (rotTal.getSensorCollection().getPulseWidthPosition() - ((MIN+MAX)/2)) / ((MAX-MIN)/2);
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        SmartDashboard.putNumber("PID in", returnPIDInput());
        SmartDashboard.putNumber("PID raw", rotTal.getSensorCollection().getPulseWidthPosition());
        SmartDashboard.putNumber("PID out", output);
        SmartDashboard.putNumber("PID set", getPIDController().getSetpoint());
        SmartDashboard.putNumber("PID err", getPIDController().getError());

        if (output > MAX_VOLTAGE)
            output = MAX_VOLTAGE;
        else if (output < -MAX_VOLTAGE)
            output = -MAX_VOLTAGE;

        // rotTal.set(ControlMode.PercentOutput, -output);
    }

    public void moveInc(double input)
    {
        input = getPIDController().getSetpoint() + (MAX_TURN_SPEED * input);
        
        if (input > 1)
            moveTo(1);
        else if (input < -1)
            moveTo(-1);
        else
            moveTo(input);
    }

    public void moveTo(double input)
    {
        setSetpoint(input);
    }

    public void set(double speed, int i)
    {
        motors[i].set(speed);
    }

    public void disable()
    {
        super.disable();

        rotTal.set(ControlMode.PercentOutput, 0);

        for (int i = 0; i < 2; i++){
            motors[i].set(0);
        }
    }

    public double getAngle()
    {
        return rotTal.getSensorCollection().getPulseWidthPosition();
    }
}
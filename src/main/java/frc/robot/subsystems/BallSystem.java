/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Ball;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallSystem extends PIDSubsystem {

    private static final double P = .05;
    private static final double I = 0;
    private static final double D = 0;

    private static final int[] DEVICE_NUMS = {0, 1, 2}; // TODO: Determine these

    private TalonSRX rotTal = new TalonSRX(DEVICE_NUMS[0]);

    private TalonSRX[] motors = {new TalonSRX(DEVICE_NUMS[1]), new TalonSRX(DEVICE_NUMS[2])};

    // private Encoder enc = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    // private CentPot enc;

    // private static final double ENC_START = 0;
    
    public BallSystem() 
    {
        super(P, I, D);
        rotTal.set(ControlMode.PercentOutput, 0);

        for (int i = 0; i < 2; i++)
            motors[i].set(ControlMode.PercentOutput, 0);
        
        // enc = new CentPot(ENC_PORT, -360, 0, ENC_START);
    }

    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Ball());    
    }

    @Override
    protected double returnPIDInput() 
    {
        return rotTal.getSensorCollection().getQuadraturePosition();
        // return enc.get();
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        rotTal.set(ControlMode.PercentOutput, output);
    }

    public void moveInc(double input)
    {
        // setSetpointRelative(input);
    }

    public void moveTo(double input)
    {
        // setSetpoint(input);
    }

    public void set(double speed, int i)
    {
        motors[i].set(ControlMode.PercentOutput, speed);
    }

    public void disable()
    {
        super.disable();
        rotTal.set(ControlMode.PercentOutput, 0);
        for (int i = 0; i < 2; i++){
            motors[i].set(ControlMode.PercentOutput, 0);
        }
    }

    public double getAngle()
    {
        return rotTal.getSensorCollection().getQuadraturePosition();
    }
}
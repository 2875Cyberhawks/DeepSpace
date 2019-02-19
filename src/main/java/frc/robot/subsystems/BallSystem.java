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
import edu.wpi.first.wpilibj.Talon;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallSystem extends PIDSubsystem {

    private static final double P = .05;
    private static final double I = 0;
    private static final double D = 0;

    private static final int[] DEVICE_NUMS = {1, 12, 10}; // TODO: Determine these - Turn, Lower, Upper

//    private TalonSRX rotTal = new TalonSRX(DEVICE_NUMS[0]);

    private Talon[] motors = {new Talon(DEVICE_NUMS[1]), new Talon(DEVICE_NUMS[2])};

    // private Encoder enc = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    // private CentPot enc;

    // private static final double ENC_START = 0;
    
    public BallSystem() 
    {
        super(P, I, D);
        // rotTal.set(ControlMode.PercentOutput, 0);

        for (int i = 0; i < 2; i++)
            motors[i].set(0);
        
        // rotTal.setInverted(false); // May need to flip this

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
        return 0;
    //    return rotTal.getSensorCollection().getQuadraturePosition();
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        // rotTal.set(ControlMode.PercentOutput, output);
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
        motors[i].set(speed);
    }

    public void disable()
    {
        super.disable();
        // rotTal.set(ControlMode.PercentOutput, 0);
        for (int i = 0; i < 2; i++){
            motors[i].set(0);
        }
    }

    public double getAngle()
    {
        return 0; 
        // return rotTal.getSensorCollection().getQuadraturePosition();
    }
}
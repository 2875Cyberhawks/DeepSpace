/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// import frc.robot.commands.Ball;
import frc.robot.util.CentPot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class BallSystem extends PIDSubsystem {

    private static final double P = 1;
    private static final double I = 1;
    private static final double D = 1;

    // private static final int[] ENC_PORTS = {0, 1};
    private static final int ENC_PORT = -99;
    private static final int[] MOTOR_PORTS = {0, 1, 2};

    private Talon rotSpark = new Talon(MOTOR_PORTS[0]);

    private Talon[] motors = {new Talon(MOTOR_PORTS[1]), new Talon(MOTOR_PORTS[2])};

    // private Encoder enc = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    private CentPot enc;

    private static final double ENC_START = 0;
    
    public BallSystem() 
    {
        super(P, I, D);
        rotSpark.set(0);

        for (int i = 0; i < 2; i++)
            motors[i].set(0);
        
        enc = new CentPot(ENC_PORT, -360, 0, ENC_START);
    }

    @Override
    public void initDefaultCommand() 
    {
        // setDefaultCommand(new Ball());    
    }

    @Override
    protected double returnPIDInput() 
    {
        return enc.get();
    }

    @Override
    protected void usePIDOutput(double output) 
    {
        rotSpark.set(output);
    }

    public void moveInc(double input)
    {
        setSetpointRelative(input);
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
        rotSpark.set(0);
        for (int i = 0; i < 2; i++){
            motors[i].set(0);
        }
    }
}
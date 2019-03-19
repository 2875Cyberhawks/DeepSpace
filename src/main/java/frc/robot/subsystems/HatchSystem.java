/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Hatch;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
    

// public class HatchSystem extends PIDSubsystem {
public class HatchSystem extends Subsystem { 


    private static final int M_PORT = 2;
    private static final int[][] SOL_PORTS = {{0, 1},{2, 3}};

    public static final double FULL_TURN = 4096;
    
    public static final double S_MIN = 1353, S_MAX = 6674; // No longer valid
    public static final double TO_MIN = 803, TO_MAX = 4518; // Distance from center point to min and max
    private double center = 2156; // to be tuned
    public boolean limited = true; 

    

    private TalonSRX motor = new TalonSRX(M_Port);
    
    private DoubleSolenoid thrustSol = new DoubleSolenoid(SOL_PORTS[0][0], SOL_PORTS[0][1]);

    public HatchSystem() 
    {

      
    }


    public void setRot(double d)
    {
        motor.set(ControlMode.PercentOutput, d);
    }

    public double setCent()
    {
        return center = motor.getSensorCollection().getPulseWidthPosition();
    }

    public double getAbs()
    {
        return motor.getSensorCollection().getPulseWidthPosition();
    }

    @Override
    public void initDefaultCommand() 
    {
        setDefaultCommand(new Hatch());
    }

    public void setSpeed(double speed)
    {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public double getCurrent()
    {
        return motor.getOutputCurrent();
    }

    public void thrust()
    {
        if (thrustSol.get() == Value.kForward)
            thrustSol.set(Value.kReverse);
        else
            thrustSol.set(Value.kForward);
    }


    public void disable()
    {
        thrustSol.close();
        motor.set(ControlMode.PercentOutput, 0);
    }
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Hatch;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

// public class HatchSystem extends PIDSubsystem {
public class HatchSystem extends Subsystem { 

    private static final int M_PORT = 2; // 1 on test bot
    private static final int[] SOL_PORT = {2, 3};

    private TalonSRX motor = new TalonSRX(M_PORT);
    private DoubleSolenoid thrustSol = new DoubleSolenoid(SOL_PORT[0], SOL_PORT[1]);

    public HatchSystem()
    {
        thrustSol.set(Value.kForward);
    }

    public void setRot(double d)
    {
        motor.set(ControlMode.PercentOutput, d);
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

    public void toggleSol()
    {
        thrustSol.set(thrustSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
    }

    public void disable()
    {
        thrustSol.close();
        motor.set(ControlMode.PercentOutput, 0);
    }
}
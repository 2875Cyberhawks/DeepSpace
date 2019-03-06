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

    private static final double P = 1.4;
    private static final double I = .001;
    private static final double D = 0;

    private static final int M_PORT = 2;
    private static final int[][] SOL_PORTS = {{0, 1},{2, 3}};

    public static final double FULL_TURN = 4096;
    
    public static final double S_MIN = 3150, S_MAX = 8630; // No longer valid
    public static final double TO_MIN = 20, TO_MAX = 20; // Distance from center point to min and max
    private double center = 6000; // to be tuned
    public boolean limited = true; 

    private TalonSRX motor = new TalonSRX(M_PORT);

    private DoubleSolenoid openSol = new DoubleSolenoid(SOL_PORTS[0][0], SOL_PORTS[0][1]);
    
    private DoubleSolenoid tiltSol = new DoubleSolenoid(SOL_PORTS[1][0], SOL_PORTS[1][1]);

    public double setpoint = 0;
    
    public HatchSystem() 
    {
        openSol.set(Value.kReverse);
        tiltSol.set(Value.kReverse);
        
        motor.configFactoryDefault();

        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        motor.setSelectedSensorPosition(0);
        
        motor.configMotionCruiseVelocity(750);
        motor.configMotionAcceleration(1500);
    
        motor.configContinuousCurrentLimit(10);
        motor.enableCurrentLimit(true);
    
        motor.configNominalOutputForward(0);
        motor.configNominalOutputReverse(0);
        motor.configPeakOutputForward(1);
        motor.configPeakOutputReverse(-1);
        motor.setIntegralAccumulator(0);
        motor.config_IntegralZone(0, 60);

        motor.config_kP(0, P);
        motor.config_kI(0, I); 
        motor.config_kD(0, D);
        motor.config_kF(0, 0);
    }

    public void setRot(double d)
    {
        motor.set(ControlMode.PercentOutput, d);
    }

    public double setCent()
    {
        return center = motor.getSelectedSensorPosition();
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

    public void moveInc(double diff) // Send in a positional difference in
    {
        if (limited)
            if (getAbs() > (center + TO_MAX) && diff > 0)
                diff = 0;
            else if (getAbs() < (center - TO_MIN) && diff < 0)
                diff = 0;
        
        SmartDashboard.putNumber("DIFF", diff);
        SmartDashboard.putNumber("RESPONSE", motor.getMotorOutputPercent());
        moveTo(setpoint + diff);
    }

    public void moveTo(double input)
    {
        motor.set(ControlMode.MotionMagic, input);
        setpoint = input;
    }

    public void toggleHatch()
    {
        openSol.set(openSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
    }

    public void toggleTilt()
    {
        tiltSol.set(tiltSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
    }

    public void disable()
    {
        openSol.close();
        tiltSol.close();
        motor.set(ControlMode.PercentOutput, 0);
    }
}
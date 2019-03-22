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

    // public static final double S_MIN = 1353, S_MAX = 6674; // No longer valid
    // public static final double TO_MIN = 803, TO_MAX = 4518; // Distance from center point to min and max
    // private double center = 2156; // to be tuned
    // public boolean limited = true; 
    public HatchSystem() 
    {}

    public void init()
    {}

    public void setRot(double d)
    {}
    
    @Override
    public void initDefaultCommand() 
    {}

    public void moveInc(double diff) // Send in a positional difference in
    {}

    public void moveTo(double input)
    {}

    public void toggleHatch()
    {}

    public void toggleTilt()
    {}

    public void disable()
    {}
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;


public class LiftSystem extends PIDSubsystem {

    private static final double P = 1;
    private static final double I = 1;
    private static final double D = 1;

    private static final int[] MOTOR_PORTS = {0, 1};

    private static final int[] ENC_PORTS = {0, 1};

    private Encoder encoder = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    
    private SpeedControllerGroup motors = new SpeedControllerGroup(new Spark(MOTOR_PORTS[0]), new Spark(MOTOR_PORTS[1]));
        
    public LiftSystem() {
        super(P, I, D);
    }

    @Override
    public void initDefaultCommand() {
        
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return 0.0;
    }

    @Override
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
}

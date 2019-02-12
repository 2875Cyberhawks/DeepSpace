/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.commands.Lift;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;


public class LiftSystem extends PIDSubsystem {

    private static final double P = 1;
    private static final double I = 1;
    private static final double D = 1;

    private static final int[] MOTOR_PORTS = {0, 1};

    private static final int[] ENC_PORTS = {2, 3};

    private Encoder encoder = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    
    private SpeedControllerGroup motors = new SpeedControllerGroup(new Spark(MOTOR_PORTS[0]), new Spark(MOTOR_PORTS[1]));

    private static final double MAX_HEIGHT = 3;
    private static final double DISTANCE_PER_PULSE = .01;
        
    public LiftSystem() {
        super(P, I, D);
        setInputRange(0, MAX_HEIGHT);
        setOutputRange(-1, 1);
        encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new Lift());
    }

    @Override
    protected double returnPIDInput() {
        return encoder.getDistance();
    }

    @Override
    protected void usePIDOutput(double output) {
        motors.set(output);
    }

    public void moveToHeight(double height){
        setSetpoint(height);
    }

    public void disable(){
        super.disable();
        motors.set(0);
    }
}

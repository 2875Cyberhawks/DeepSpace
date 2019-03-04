package frc.robot.subsystems;

import frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimbSystem extends Subsystem {
 
    private static final int MOT_PORT = 10; // 0 on expansion board
    private static final int SOL_PORT = 1;

    private Spark motor = new Spark(MOT_PORT);

    private Solenoid sol = new Solenoid(SOL_PORT);

    public void initDefaultCommand() {
        setDefaultCommand(new Climb());
    }

    public void setMotor(double speed){
        motor.set(speed);
    }

    public void toggleSol(){
        sol.set(!sol.get());
    }

    public void disable(){
        motor.set(0);
        sol.free();
    }
}

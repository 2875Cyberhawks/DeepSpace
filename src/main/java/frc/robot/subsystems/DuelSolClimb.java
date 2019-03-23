package frc.robot.subsystems;

import frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class DuelSolClimber extends Subsystem
{

    private static final int MOT_PORT = 10; // 0 on expansion board
    private static final int SOL_PORTS[] = {1, 2, 3}; // First two for doubleSol, second for single sol

    private Spark motor = new Spark(MOT_PORT);

    private DoubleSolenoid lockSol = new DoubleSolenoid(SOL_PORT[0],
                                                        SOL_PORT[1]);

    private Solenoid lowSol = new Solenoid(SOL_PORT[2]);

    public void initDefaultCommand()
    {
        setDefaultCommand(new Climb());
    }

    public void setMotor(double speed)
    {
        motor.set(speed);
    }

    public void toggleLock()
    {
        lockSol.set(!sol.get());
    }

    public void toggleLow()
    {
        lockSol.set(!lockSol.get());
    }

    public void disable()
    {
        motor.set(0);
        lockSol.free();
        lowSol.free();
    }
}

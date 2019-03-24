package frc.robot.subsystems;

import frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClimbSystem extends Subsystem
{
    private static final int MOT_PORT = 10; // 0 on expansion board
    private static final int SOL_PORTS[] = {1, 2};

    private Spark motor = new Spark(MOT_PORT);

    private DoubleSolenoid lockSol = new DoubleSolenoid(SOL_PORTS[0],
                                                        SOL_PORTS[1]);
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
        lockSol.set(lockSol.get() == Value.kForward ?
                        Value.kReverse :
                        Value.kForward);
    }

    public void disable()
    {
        motor.set(0);
        lockSol.free();
    }
}

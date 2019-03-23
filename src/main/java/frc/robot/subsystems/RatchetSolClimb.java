package frc.robot.subsystems;

import frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// TODO: Doesn't work right now - needs a moveLower method
public class RatchetSolClimb extends Subsystem
{

    private static final int MOT_PORT = 10; // 0 on expansion board
    private static final int SOL_PORTS[] = {1, 2};

    private Spark motor = new Spark(MOT_PORT);

    private DoubleSolenoid lockSol = new DoubleSolenoid(SOL_PORT[0],
                                                        SOL_PORT[1]);
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
        lockSol.set(thrustSol.get() == Value.kForward ?
                        Value.kReverse :
                        Value.kForward);
    }

    public void disable()
    {
        motor.set(0);
        lockSol.free();
        lowSol.free();
    }
}

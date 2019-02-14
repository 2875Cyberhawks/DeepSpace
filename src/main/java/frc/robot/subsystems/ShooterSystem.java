
package frc.robot.subsystems;

import frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Talon;


public class ShooterSystem extends Subsystem {

    private static final int[] MOTOR_PORTS = {3, 4, 5};
    
    private Talon[] motors = {new Talon(MOTOR_PORTS[1]), new Talon(MOTOR_PORTS[2]), new Talon(MOTOR_PORTS[3])};

    
    public ShooterSystem(){
      for (int i = 0; i < 3; i++){
        motors[i].set(0);
      }
    }

    public void initDefaultCommand() {
        setDefaultCommand(new Shoot());
    }

    public void setSpeed(double speed, int i){
      motors[i].set(speed);
    }

    public void disable(){
      for (int i = 0; i < 3; i++){
        motors[i].set(0);
    }
    }
}

// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
// /* Open Source Software - may be modified and shared by FRC teams. The code     */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                                                                                             */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// import frc.robot.commands.Hatch;

// import edu.wpi.first.wpilibj.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.Talon;
// import edu.wpi.first.wpilibj.Solenoid;
// import edu.wpi.first.wpilibj.Encoder;
    

// public class HatchSystem extends PIDSubsystem {

//     private static final double P = 1;
//     private static final double I = 1;
//     private static final double D = 1;

//     private static final int M_PORT = 0;
//     private static final int[] SOL_PORTS = {1, 2};
//     private static final int[] ENC_PORTS = {3, 4};

//     private Talon motor = new Talon(M_PORT);

//     private Solenoid openSol = new Solenoid(SOL_PORTS[0]);
//     private Solenoid tiltSol = new Solenoid(SOL_PORTS[1]);

//     private Encoder enc = new Encoder(ENC_PORTS[0], ENC_PORTS[1]);
    
//     public HatchSystem() {
//         super(P, I, D);
//         setInputRange(-1, 1);
//         setOutputRange(-1, 1);
//     }

//     @Override
//     public void initDefaultCommand() {
//         setDefaultCommand(new Hatch());
//     }

//     @Override
//     protected double returnPIDInput() {
//         return enc.get(); //may use a more specific method
//     }

//     @Override
//     protected void usePIDOutput(double output) {
//         motor.set(output);
//     }

//     public void move(double input){
//         setSetpointRelative(input);
//     }

//     public void toggleHatch(){
//         openSol.set(!openSol.get());
//     }

//     public void toggleTilt(){
//         tiltSol.set(!tiltSol.get());
//     }

//     public void disable(){
//         super.disable();
//         openSol.free();
//         tiltSol.free();
//         motor.set(0);
//     }
// }

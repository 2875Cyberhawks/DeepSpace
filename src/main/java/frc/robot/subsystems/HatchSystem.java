// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
// /* Open Source Software - may be modified and shared by FRC teams. The code     */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                                                                                             */
// /*----------------------------------------------------------------------------*/

// package frc.robot.subsystems;

// // import frc.robot.commands.Hatch;

// import edu.wpi.first.wpilibj.command.PIDSubsystem;
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
    

// public class HatchSystem extends PIDSubsystem {

//     private static final double P = 1;
//     private static final double I = 1;
//     private static final double D = 1;

//     private static final int M_PORT = 0;
//     private static final int[][] SOL_PORTS = {{0, 1},{2, 3}};
//     private static final int[] ENC_PORTS = {3, 4};

//     private TalonSRX motor = new TalonSRX(M_PORT);

//     private DoubleSolenoid openSol = new DoubleSolenoid(SOL_PORTS[0][0], SOL_PORTS[0][1]);
//     private boolean openSolOpen = false;

//     private DoubleSolenoid tiltSol = new DoubleSolenoid(SOL_PORTS[1][0], SOL_PORTS[1][1]);
//     private boolean tiltSolOpen = false;

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
//         return motor.getSensorCollection().getQuadraturePosition();
//     }

//     @Override
//     protected void usePIDOutput(double output) {
//         motor.set(ControlMode.PercentOutput, output);
//     }

//     public void moveInc(double input){
//         setSetpointRelative(input);
//     }

//     public void moveTo(double input){
//         setSetpoint(input);
//     }

//     public void toggleHatch()
//     {
//         openSol.set(openSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
//     }

//     public void toggleTilt()
//     {
//         tiltSol.set(tiltSol.get() == Value.kForward ? Value.kReverse : Value.kForward);
//     }

//     public void disable()
//     {
//         super.disable();
//         openSol.free();
//         tiltSol.free();
//         motor.set(ControlMode.PercentOutput, 0);
//     }
// }
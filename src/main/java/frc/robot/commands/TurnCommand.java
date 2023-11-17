// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.DriveBase;
// import frc.robot.constants.PID_IDConstants;

// public class TurnCommand extends CommandBase {
//   double setpoint;
//   DriveBase drivebase;
//   ProfiledPIDController controller;

//   /** Creates a new turnCommandPID. */
//   public TurnCommand(DriveBase drivebase, double setpoint) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.setpoint = setpoint;

//     addRequirements(drivebase);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     double kP = PID_IDConstants.turnCommand.kP;
//     double kI = PID_IDConstants.turnCommand.kI;
//     double kD = PID_IDConstants.turnCommand.kD;

//     int maxVelocity = PID_IDConstants.turnCommand.maxVelocity;
//     int maxAcceleration = PID_IDConstants.turnCommand.maxAcceleration;

//     TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(maxVelocity,maxAcceleration);
  
//     controller = new ProfiledPIDController(kP, kI, kD, profile);

//     controller.enableContinuousInput(-Math.PI, Math.PI);

//     controller.setTolerance(2);

//     controller.setGoal(setpoint);

//     controller.reset(drivebase.getRotationDeg());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double calc = controller.calculate(drivebase.getRotationDeg(), setpoint);
//     drivebase.autonomousArcadeDrive(0, calc);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drivebase.autonomousArcadeDrive(0, 0);
//     new InstantCommand(() -> {System.out.println("Turn PID Command Done!");});
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

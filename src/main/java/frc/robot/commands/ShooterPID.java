// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.constants.PID_IDConstants.aimHood;
// import frc.robot.constants.PID_IDConstants.rotaterHood;
// import frc.robot.subsystems.ShooterHood;

// public class ShooterPID extends CommandBase {
//   /** Creates a new shooterPID. */
//   ProfiledPIDController rotaterPID;
//   ProfiledPIDController aimPID; 
//   double rotateSetpoint;
//   double aimSetpoint;
//   ShooterHood shooter;
  

//   public ShooterPID(ShooterHood shooter, double rotateSetpoint, double aimSetpoint) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.rotateSetpoint = rotateSetpoint;
//     this.aimSetpoint = aimSetpoint;
//     addRequirements(shooter);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     int krP = rotaterHood.kP;
//     int krI = rotaterHood.kI;
//     int krD = rotaterHood.kD;

//     int maxVelocityR = rotaterHood.maxVelocity;
//     int maxAccelerationR  = rotaterHood.maxAcceleration;

//     TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(maxVelocityR,maxAccelerationR);

//     rotaterPID = new ProfiledPIDController(krP , krI, krD, profile);
    
//     rotaterPID.setGoal(rotateSetpoint);


//     rotaterPID.setTolerance(2);

//     rotaterPID.reset(shooter.getRotationDegrees());

//     double kaP = aimHood.kP;
//     double kaI = aimHood.kI;
//     double kaD = aimHood.kD;

//     int maxVelocity = aimHood.maxVelocity;
//     int maxAcceleration = aimHood.maxAcceleration;

//     TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration);

//     aimPID = new ProfiledPIDController(kaP, kaI, kaD, constraints);

//     aimPID.setGoal(aimSetpoint);

//     aimPID.setTolerance(2);

//     aimPID.reset(shooter.getAimDegrees());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double aimCalc = aimPID.calculate(shooter.getAimDegrees(), aimSetpoint);
//     double rotateCalc = aimPID.calculate(shooter.getRotationDegrees(), rotateSetpoint);

//     shooter.driveHood(rotateCalc, aimCalc);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     shooter.driveHood(0, 0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }

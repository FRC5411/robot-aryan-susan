// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterHood;

public class turretPos extends CommandBase {

  ShooterHood shooter;
  Supplier<Double> rotationFunction, aimFunction;

  /** Creates a new turretPos. */
  public turretPos(ShooterHood shooter, Supplier<Double> rotationFunction, Supplier<Double> aimFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.rotationFunction = rotationFunction;
    this.aimFunction = aimFunction;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotate = rotationFunction.get();
    double aim = aimFunction.get();

    shooter.isSafe2MoveRotate(rotate);
    shooter.isSafe2MoveRotate(aim);

    shooter.driveHood(rotate, aim);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.driveHood(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
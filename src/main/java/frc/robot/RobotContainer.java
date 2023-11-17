// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arcadeDrive;
import frc.robot.commands.turretPos;
import frc.robot.subsystems.*;

public class RobotContainer {

  CommandXboxController driveController = new CommandXboxController(0);
  CommandXboxController utilityController = new CommandXboxController(1);
  DriveBase drivebase;
  Intake intake;
  BallHandler handle;
  ShooterHood shooter;
  public static Boolean runMotor;
  
  public RobotContainer() {
  
    drivebase = new DriveBase();

    intake = new Intake();

    handle = new BallHandler();

    shooter = new ShooterHood();

    runMotor = false;
    
    // Drive //
    drivebase.setDefaultCommand(new arcadeDrive(drivebase, 
    () -> (-driveController.getLeftY()) , 
    () -> (driveController.getRightX()) * -1
    ));

    // Turret Aim // 
    shooter.setDefaultCommand(new turretPos(shooter, 
    () -> (-shooter.filterRotate(utilityController.getRightX())), 
    () -> (-shooter.filterAim(utilityController.getLeftY()
    ))));

    // Shooter //
    shooter.runShooter();
  
    configureBindings();
    
  }

  private void configureBindings() {

    utilityController.leftTrigger().onTrue(new InstantCommand(() -> intake.grab()));
    utilityController.leftTrigger().onFalse(new InstantCommand(() -> intake.stopIntake()));

    utilityController.leftTrigger().onTrue(new InstantCommand(() -> intake.retract()));
    utilityController.leftTrigger().onFalse(new InstantCommand(() -> intake.stopIntake()));

    utilityController.rightBumper().onTrue(new InstantCommand(() -> handle.ball2Shooter()));
    utilityController.rightBumper().onFalse(new InstantCommand(() -> handle.offBallHandler()));

    utilityController.leftTrigger().onTrue(new InstantCommand(() -> handle.exitShooter()));
    utilityController.leftTrigger().onFalse(new InstantCommand(() -> handle.offBallHandler()));

    utilityController.a().onTrue(new InstantCommand(() -> shooter.toggleBoolean()));
  

    while(true){
      shooter.runShooter();
    }
  }

  public void periodic(){}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

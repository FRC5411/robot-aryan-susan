// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class DriveBase extends SubsystemBase {
  /** Creates a new driveBase. */
  private WPI_TalonSRX rightLead;
  private WPI_TalonSRX rightBack;
  private WPI_TalonSRX leftLead;
  private WPI_TalonSRX leftBack;
  private DifferentialDrive drive;
  private Pose2d pose2d;
  
  public DriveBase() {
    rightLead = new WPI_TalonSRX(constants.DriveConstants.K_RIGHTLEAD);
    rightBack = new WPI_TalonSRX(constants.DriveConstants.K_RIGHTBACK);
    leftLead = new WPI_TalonSRX(constants.DriveConstants.K_LEFTLEAD);
    leftBack = new WPI_TalonSRX(constants.DriveConstants.K_LEFTBACK);

    pose2d = new Pose2d();

    configMotor(rightLead);
    configMotor(rightBack);
    configMotor(leftLead);
    configMotor(leftBack);

    rightBack.follow(rightLead);
    leftBack.follow(leftLead);

    rightBack.setInverted(true);
    rightLead.setInverted(true);

    drive = new DifferentialDrive(leftLead, rightLead);

    rightLead.configNeutralDeadband(0.1);
    rightBack.configNeutralDeadband(0.1);
    leftLead.configNeutralDeadband(0.1);
    leftBack.configNeutralDeadband(0.1);

  }

  public void setMotors(double speed, double turn){
    drive.arcadeDrive(speed, turn);
  }

  public double applyDeadzone(double input){
    if(input > 0.1){
      return input;}

    else{
      return 0;}

  }

  public double getX(){
    return pose2d.getX();
  }

  public double getY(){
    return pose2d.getY();
  }

  public void autonDriveCommand(double speed, double turn){
    drive.arcadeDrive(speed, turn);
    drive.feed();
  }

  public double getRotationDeg(){
    return pose2d.getRotation().getDegrees();
  }

  public void autonomousArcadeDrive(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
    drive.feed();
  }
  
  public void configMotor(WPI_TalonSRX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Rotation: ", getRotationDeg());
  }
}

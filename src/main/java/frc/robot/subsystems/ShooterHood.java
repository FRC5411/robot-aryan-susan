// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.ShooterConstants;

public class ShooterHood extends SubsystemBase {

  private WPI_TalonSRX aimHood;
  private WPI_TalonFX shooterRight;
  private WPI_TalonFX shooterLeft;
  private WPI_TalonSRX rotaterHood;

  private double kTicks2Degree;
  private double kgearRatioRotation;
  private double kgearRatioAim;


  /** Creates a new FlyWheel. */
  public ShooterHood() {

    aimHood = new WPI_TalonSRX(ShooterConstants.K_AIMHOOD);
    shooterRight = new WPI_TalonFX(ShooterConstants.K_SHOOTERRIGHT);
    shooterLeft = new WPI_TalonFX(ShooterConstants.K_SHOOTERLEFT);
    rotaterHood = new WPI_TalonSRX(ShooterConstants.K_ROTATERHOOD);

    kTicks2Degree = 1/1024.0 * 360.0;

    kgearRatioRotation = ShooterConstants.K_ROTATER_GEARRATIO_ROTATION;
    kgearRatioAim = ShooterConstants.K_ROTATER_GEARRATIO_AIM;

    configMotor(aimHood);
    configMotor(shooterRight);
    configMotor(shooterLeft);
    configMotor(rotaterHood);

    shooterRight.setInverted(true);

    aimHood.setInverted(false);
    rotaterHood.setInverted(true);

    shooterRight.follow(shooterLeft);

    rotaterHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    aimHood.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

    rotaterHood.configNeutralDeadband(0.1);
    aimHood.configNeutralDeadband(0.1);
  }

  // Hood Shooter Angle PID Methods // 
  public void setAimHood(double speed){
    aimHood.set(speed);
  }

  // Rotation Hood PID Methods // 
  public void setRotaterHood(double speed){
    rotaterHood.set(speed);
  }

  // XBOX Command Drive // 

  public void aimStop(){
    aimHood.set(0);
  }

  public void rotateOff(){
    rotaterHood.set(0);
  }

  public void runShooter(){
    while(RobotContainer.runMotor){    
      shooterRight.set(0.8);
      shooterLeft.set(0.8);}

    while(!RobotContainer.runMotor){
      offShooter();
    }
  }

  public double getRotationDegrees(){
    return (rotaterHood.getSelectedSensorPosition() * kTicks2Degree / kgearRatioRotation) - ShooterConstants.K_ROTATERHOOD_OFFSET;
  }

  public double getAimDegrees(){
    return (aimHood.getSelectedSensorPosition() * kTicks2Degree / kgearRatioAim) - ShooterConstants.K_AIMHOOD_OFFSET;
  }

  public void toggleBoolean(){
    RobotContainer.runMotor = !RobotContainer.runMotor;
  }

  public void offShooter(){
    shooterRight.set(0);
    shooterLeft.set(0);
  }

  public void driveHood(double rotation, double aim){
    rotaterHood.set(rotation);
    aimHood.set(aim);
  }

  public double filterAim(double speedPar){
    // First it checks if speedPar is 0, if it is then it stays 0 //
    // If sppedPar is greater than 0 it becomes 0.4, and if it less than 0 it becomes - 0.4 //
    double constantSpeed = ShooterConstants.K_AIMHOOD_CONSTANT_SPEED; 
    
    if(speedPar == 0){
      return 0;}

    else if(speedPar != 0){
      if(speedPar > 0){
        return constantSpeed;}

      else{
        return -constantSpeed;}}
    return 0;
  }

  public double filterRotate(double speedPar){
    // First it checks if speedPar is 0, if it is then it stays 0 //
    // If sppedPar is greater than 0 it becomes 0.4, and if it less than 0 it becomes - 0.4 //
    double constantSpeed = ShooterConstants.K_ROTATERHOOD_CONSTANT_SPEED; 
    
    if(speedPar == 0){
      return 0;}

    else if(speedPar != 0){
      if(speedPar > 0){
        return constantSpeed;}

      else{
        return -constantSpeed;}}
    return 0;
  }

  public double isSafe2MoveRotate(double rotatePar){
    if(ShooterConstants.K_ROTATE_LEFT_MAX <= getRotationDegrees() && getRotationDegrees() <= ShooterConstants.K_ROTATE_RIGHT_MAX){
      return rotatePar;
    }

    else{
      return 0.0;
    }
  }

  public double isSafe2MoveAim(double aimPar){
    if(ShooterConstants.K_AIM_DOWN_MAX <= getAimDegrees() && getAimDegrees() <= ShooterConstants.K_AIM_UP_MAX){
      return aimPar;
    }

    else{
      return 0.0;
    }
  }

  public void configMotor(WPI_TalonSRX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void configMotor(WPI_TalonFX motor){
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Roatater Hood Degrees " , getRotationDegrees());
    SmartDashboard.putNumber("Aim Hood Degrees " , getAimDegrees());
  }
}

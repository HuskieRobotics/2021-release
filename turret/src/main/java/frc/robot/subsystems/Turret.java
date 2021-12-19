/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends SubsystemBase {
  private TalonSRX baseRotationMotor;
  private CANSparkMax hoodMotor;
  private TalonFX shootMotor1;
  private TalonFX shootMotor2;
  private double shooterPower;
 
  /**
   * Creates a new Turret.
   */
  public Turret() {
    this.baseRotationMotor = new TalonSRX(TurretConstants.baseRotationMotorID);
    this.hoodMotor = new CANSparkMax(TurretConstants.hoodMotorID, MotorType.kBrushless);
    this.shootMotor1 = new TalonFX(TurretConstants.shootMotor1ID);
    this.shootMotor2 = new TalonFX(TurretConstants.shootMotor2ID);
    
    this.baseRotationMotor.setNeutralMode(NeutralMode.Brake);
    this.hoodMotor.setIdleMode(IdleMode.kBrake);
  }

  public double getLimelightX() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public double getLimelightY() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }
  
  public double getLimelightArea() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  }

  public double getCurrentBasePosition() {
    return this.baseRotationMotor.getSelectedSensorPosition();
  }

  // TODO adjust this for CANSparkMax encoder
  public double getCurrentHoodPosition() {
    return this.hoodMotor.getEncoder().getPosition();
  }

  public void setBasePower(double power) {
    // Slowdown close to maximum (150 ticks away)
    if((this.getCurrentBasePosition() > TurretConstants.BASE_MAX - 150 || this.getCurrentBasePosition() < -TurretConstants.BASE_MAX + 150)
        && (this.getCurrentBasePosition() < TurretConstants.BASE_MAX && this.getCurrentBasePosition() > -TurretConstants.BASE_MAX)) {
      double powerMultiplier = (1/150) * Math.abs(TurretConstants.BASE_MAX - this.getCurrentBasePosition());
      this.baseRotationMotor.set(ControlMode.PercentOutput, powerMultiplier * power);
    }
    // If we are not at or past maximum and are greater than 150 ticks away from maximum, set motor power
    else if(this.getCurrentBasePosition() < TurretConstants.BASE_MAX && this.getCurrentBasePosition() > -TurretConstants.BASE_MAX) {
      this.baseRotationMotor.set(ControlMode.PercentOutput, power);
    }
    // If we are at or past maximum, don't move the base
    else {
      this.baseRotationMotor.set(ControlMode.PercentOutput, 0);
    }
      
  }
  
  public void setHoodPower(double power) {
    if((this.getCurrentHoodPosition() > TurretConstants.HOOD_MAX - 3 || this.getCurrentHoodPosition() < 0 + 3)
        && (this.getCurrentHoodPosition() < TurretConstants.HOOD_MAX && this.getCurrentHoodPosition() > -TurretConstants.HOOD_MAX)) {
      double powerMultiplier = (1/3) * Math.abs(TurretConstants.HOOD_MAX - this.getCurrentHoodPosition());
      this.hoodMotor.set(powerMultiplier * power);
    }
    // If we are not at or past maximum and are greater than some number of ticks away from maximum, set motor power
    else if(this.getCurrentHoodPosition() < TurretConstants.HOOD_MAX && this.getCurrentHoodPosition() > -TurretConstants.HOOD_MAX) {
      this.hoodMotor.set(power);
    }
    // If we are at or past maximum, don't move the hood
    else {
      this.hoodMotor.set(0);
    }
  }
  
  /*
  public void setShooterPower(double power) {
    this.shootMotor1.set(ControlMode.PercentOutput, power);
    this.shootMotor2.set(ControlMode.PercentOutput, -power);
  }
  */

  public void setShooterPower() {
    this.shootMotor1.set(ControlMode.PercentOutput, this.shooterPower);
    this.shootMotor2.set(ControlMode.PercentOutput, -this.shooterPower);
  }
  public void setShooterPower(double power) {
    this.shootMotor1.set(ControlMode.PercentOutput, power);
    this.shootMotor2.set(ControlMode.PercentOutput, -power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Adjust things with SmartDashboard variables
    this.shooterPower = SmartDashboard.getNumber("Shooter Power", this.shooterPower);
    SmartDashboard.putNumber("Shooter Power", this.shooterPower);

    TurretConstants.BASE_P = SmartDashboard.getNumber("Base P",TurretConstants.BASE_P);
    SmartDashboard.putNumber("Base P", TurretConstants.BASE_P);

    TurretConstants.HOOD_P = SmartDashboard.getNumber("Hood P", TurretConstants.HOOD_P);
    SmartDashboard.putNumber("Hood P", TurretConstants.HOOD_P);

    TurretConstants.BASE_MAX = (int) SmartDashboard.getNumber("Base Max", TurretConstants.BASE_MAX);
    SmartDashboard.putNumber("Base Max", TurretConstants.BASE_MAX);

    TurretConstants.HOOD_MAX = (int) SmartDashboard.getNumber("Hood Max", TurretConstants.HOOD_MAX);
    SmartDashboard.putNumber("Hood Max", TurretConstants.HOOD_MAX);
  }
}
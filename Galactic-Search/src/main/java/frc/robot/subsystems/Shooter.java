package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  
  private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1CanID);
  private TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2CanID);

  public Shooter () {
    shooterMotor1.setNeutralMode(NeutralMode.Brake);
    shooterMotor2.setNeutralMode(NeutralMode.Brake);
    shooterMotor1.setInverted(true);
  }
  
  public void setPower (double power) {
    shooterMotor1.set(ControlMode.PercentOutput, power);
    shooterMotor2.set(ControlMode.PercentOutput, power);
    SmartDashboard.putNumber("shooting now",SmartDashboard.getNumber("shooting now", 0)+1);
  }

  public void periodic(){
    ShooterConstants.shootingSpeed = SmartDashboard.getNumber("shooter power", ShooterConstants.shootingSpeed);
    SmartDashboard.putNumber("shooter power", ShooterConstants.shootingSpeed);
  }

   
}
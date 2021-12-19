/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  private TalonFX[] leftMotors;
  private TalonFX[] rightMotors;

  // encoders tell how many rotations that motors have made
  private CANCoder leftEncoder;
  private CANCoder rightEncoder;

  private boolean isMotorPowersInverted;

  // filter is for trapezoidal calculations
  private SlewRateLimiter filter;

  // object that stores boolean value
  private Solenoid gearShifter;

  private PigeonIMU pigeon;

  private double currentLeftPower = 0;
  private double currentRightPower = 0;

  public Drivetrain() {
    // initialize motor arrays
    leftMotors = new TalonFX[DriveConstants.DRIVETRAIN_LEFT_TALONS.length];
    rightMotors = new TalonFX[DriveConstants.DRIVETRAIN_RIGHT_TALONS.length];

    // initialize each motor with each CANID from DRIVETRAIN_LEFT&RIGHT_TALONS
    for (int i = 0; i < DriveConstants.DRIVETRAIN_LEFT_TALONS.length; i++) {
      leftMotors[i] = new TalonFX(DriveConstants.DRIVETRAIN_LEFT_TALONS[i]);
      rightMotors[i] = new TalonFX(DriveConstants.DRIVETRAIN_RIGHT_TALONS[i]);

      // sets talon's neutral mode to brake mode
      this.rightMotors[i].setNeutralMode(NeutralMode.Brake);
      this.rightMotors[i].setInverted(true); // inverts right motors

      this.leftMotors[i].setNeutralMode(NeutralMode.Brake);
    }

    this.gearShifter = new Solenoid(RobotConstants.COMPRESSOR_CANID, DriveConstants.GEAR_SHIFT_CHANNEL); // initialize solenoid

    // initialize encoders
    this.leftEncoder = new CANCoder(DriveConstants.LEFT_ENCODER_CANID);
    this.rightEncoder = new CANCoder(DriveConstants.RIGHT_ENCODER_CANID);

    filter = new SlewRateLimiter(DriveConstants.RATE_LIMIT); // initialize filter, RATE_LIMIT is what limits rate that joystick values change (trapezoidal)
    
    this.isMotorPowersInverted = true;
 
    this.pigeon = new PigeonIMU(DriveConstants.PIEGON_CANID);
    this.pigeon.setYaw(0);
 
  }



  /**
   * Converts x,y joystick values to left,right motor outputs & sets motor power
   * joystick y -> forward, backward
   * joystick x -> turning
   */
  public void arcadeDrive(double joystickX, double joystickY) {
    // code with trapezoidal calculations
    // joystickX = filter.calculate(changeValueByGear(deadZone(joystickX), gearShifter.get(), false));
    // joystickY = filter.calculate(changeValueByGear(invertTranslation(deadZone(joystickY)), gearShifter.get(), true));

    // code without trapezoidal calculations
    joystickX = changeValueByGear(deadZone(joystickX), gearShifter.get(), false);
    joystickY = changeValueByGear(invertIfNeeded(deadZone(joystickY)), gearShifter.get(), true);

    double leftMotorOutput = (joystickX + joystickY);
    double rightMotorOutput = (joystickY - joystickX);
    setPower(leftMotorOutput, rightMotorOutput);
  }

  /**
   * sets joystick x,y to 0 if less than deadzone value
   */
  public double deadZone(double joystickVal) {
    if (Math.abs(joystickVal) < DriveConstants.DEADZONE) return 0.0;
    return joystickVal;
  }

  /**
   * inverts joystick y value if isMotorPowersInverted == true
   */
  public double invertIfNeeded(double joystickY) {
    return this.isMotorPowersInverted ? joystickY * -1 : joystickY;
  }

  /**
   * modifies joystick value (x or y depending on isJoystickY) based on isHighGear by raising it to an exponent
   * exponent is determined by choosing between LOWGEAR/HIGHGEAR & TURNING/TRANSLATION constants
   */
  public double changeValueByGear(double joystickVal, boolean isHighGear, boolean isJoystickY) {
    double exponent = (isHighGear ? DriveConstants.HIGHGEAR_TURNING_POWER : DriveConstants.LOWGEAR_TURNING_POWER);
    if (isJoystickY) exponent = (isHighGear ? DriveConstants.HIGHGEAR_TRANSLATION_POWER : DriveConstants.LOWGEAR_TRANSLATION_POWER);    
    return (joystickVal < 0 ? -1*Math.abs(Math.pow(joystickVal, exponent)) : Math.abs(Math.pow(joystickVal, exponent)));
  }

  /**
   * Caps the acceleration of the robot at the value of trapezoidal (unused method that shows old way of calculating trapezoidal, doesn't use filter)
   */
  public double changeByTrapezoidal(double prevMotorOutput, double motorOutput) {
    if (motorOutput > prevMotorOutput) return ((motorOutput - prevMotorOutput) > DriveConstants.TRAPEZOIDAL) ? prevMotorOutput + DriveConstants.TRAPEZOIDAL : motorOutput;
    return ((prevMotorOutput - motorOutput) > DriveConstants.TRAPEZOIDAL) ? prevMotorOutput - DriveConstants.TRAPEZOIDAL : motorOutput;
  }

  /**
   * sets the power for each individual left & right motor
   */
  public void setPower(double leftPower, double rightPower) {
    for (int i=0; i<DriveConstants.DRIVETRAIN_LEFT_TALONS.length; i++) {
      leftMotors[i].set(ControlMode.PercentOutput, leftPower);
      rightMotors[i].set(ControlMode.PercentOutput, rightPower);
    }
  }

  /**
   * Gets the left distance from encoder in feet
   */
  public double getLeftDistance() {
    return leftEncoder.getPosition() * DriveConstants.ENCODER_DISTANCE_PER_COUNT_FEET;
  }

  /**
   * Gets the right distance from encoder in feet
   */
  public double getRightDistance() {
    return rightEncoder.getPosition() * DriveConstants.ENCODER_DISTANCE_PER_COUNT_FEET;
  }

  public double getHeadingTicks() {
    return (this.getHeadingDegrees() * DriveConstants.DEGREES_TO_TICKS); 
  }
  
  /**
   * Gets the angle out robot is facing, in degrees
   * @return the angle of the robot in degrees
   */
  public double getHeadingDegrees() {
    double[] ypr = new double[3];
    this.pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
  * resets the positions of the encoders
  */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * switches boolean isMotorPowersInverted from true to false or false to true when called
   */
  public void invertMotors() {
    this.isMotorPowersInverted = !this.isMotorPowersInverted;
  }

  /**
   * sets boolean value on solenoid object
   * if isHighGear true: highgear
   * if isHighGear false: lowgear
   */
  public void setGear(boolean isHighGear) {
    gearShifter.set(isHighGear);
  }

  public void setPowerAuto(double leftPower, double rightPower, double maxPower) {
    leftPower = Math.min(maxPower, leftPower);
    leftPower = Math.max(-maxPower, leftPower);
    rightPower = Math.min(maxPower, rightPower);
    rightPower = Math.max(-maxPower, rightPower);

    for(int i = 0; i < 3; i++){
      this.currentLeftPower = this.changeByTrapezoidal(this.currentLeftPower, leftPower);
      this.currentRightPower = this.changeByTrapezoidal(this.currentRightPower, rightPower);
      this.leftMotors[i].set(ControlMode.PercentOutput, this.currentLeftPower);
      this.rightMotors[i].set(ControlMode.PercentOutput, this.currentRightPower);
    }
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw Degrees", getHeadingDegrees());
    SmartDashboard.putNumber("Yaw Ticks", getHeadingTicks());
    AutoConstants.P_TURN = SmartDashboard.getNumber("Turning P", AutoConstants.P_TURN);
    SmartDashboard.putNumber("Turning P", AutoConstants.P_TURN);
    SmartDashboard.putNumber("Distance (ft.)", this.getLeftDistance());
  }
}

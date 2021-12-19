/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants.TurretConstants;

/**
 * An example command that uses an example subsystem.
 */
public class TurnBaseToSetpoint extends PIDCommand {

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnBaseToSetpoint(Turret turret, int setpoint) {
    super(
      new PIDController(TurretConstants.BASE_P, 0, 0),
      turret::getLimelightX,
      setpoint,
      output -> turret.setBasePower(output),
      turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

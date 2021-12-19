package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.controller.PIDController;

import frc.robot.Constants.TurretConstants;

public class TurnHoodToSetpoint extends PIDCommand{
    public TurnHoodToSetpoint(Turret turret, int setpoint) {
        super(
            new PIDController(TurretConstants.HOOD_P, 0, 0), 
            turret::getLimelightY, 
            setpoint, 
            output -> turret.setHoodPower(output), 
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
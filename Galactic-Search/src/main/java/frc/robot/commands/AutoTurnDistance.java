package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoTurnDistance extends PIDCommand {
    public AutoTurnDistance(double angle, Drivetrain drive) {
        super(
            new PIDController(AutoConstants.P_TURN, 0, 0),
            drive::getHeadingTicks,
            (drive.getHeadingTicks() + angle * (DriveConstants.DEGREES_TO_TICKS)),
            output -> drive.setPowerAuto(-output, output, 0.7),
            drive
            );
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return this.getController().atSetpoint();
    }
}
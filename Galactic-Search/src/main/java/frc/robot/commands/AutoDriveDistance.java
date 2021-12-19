package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveDistance extends PIDCommand {
    public AutoDriveDistance(double distance, Drivetrain drive) {
        super(
          new PIDController(AutoConstants.P_DRIVE, 0, 0),
            drive::getLeftDistance,
            drive.getLeftDistance() + distance,
            output -> drive.setPowerAuto(output, output, 1),
            drive
        );
        this.getController().setTolerance(AutoConstants.DRIVE_TOLERANCE);
    }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return this.getController().atSetpoint();
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;

public class LimelightCenter extends PIDCommand {
    public LimelightCenter(Drivetrain drive) {
        super(
            new PIDController(AutoConstants.P_LIMELIGHTTURN, 0, 0),
            drive::getLimelightX,
            0,
            output -> drive.setPowerAuto(-output, output, 0.7),
            drive
        );
        this.getController().setTolerance(AutoConstants.LIME_TOLERANCE);
    } 
}

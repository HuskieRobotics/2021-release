package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.SwerveSimRunner;

public class SwerveSimCommand extends CommandBase {
    @Override
    public void initialize() {
        java.awt.EventQueue.invokeLater(new Runnable() {
            public void run() {
                SwerveSimRunner frame = new SwerveSimRunner();
            }
        });
    }
}

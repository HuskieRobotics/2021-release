package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.RobotGlobal;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSetPower extends CommandBase {

  private Collector c; 
  private Shooter sh;
  private Storage st;
  private double stPower;
  private double shPower;

  public LimelightSetPower(Collector collector, Storage storage, Shooter shooter, double storagePower) {
    shPower = 0.16*(Math.pow(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0),2))-0.6*(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0))+1.03;
    stPower = storagePower;
    c = collector;
    sh = shooter;
    st = storage;
    addRequirements(c,st,sh);
    /*
    addCommands(
                new RunCommand(() -> collector.idle(), c),
                new RunCommand(() -> shooter.setPower(shooterPower), sh), 
                new RunCommand(() -> storage.setPower(storagePower), st));
    //SmartDashboard.putNumber("shoot init", 0);
    */
  }

  @Override
  public void initialize() {
    RobotGlobal.state = "Shooting";
    c.idle();
  }

  @Override
  public void execute() {
    st.setPower(stPower);
    sh.setPower(shPower);
    c.idle();

  }

  @Override
  public void end(boolean interrupted) {
    RobotGlobal.state = "Idle";
    st.setPower(0);
    sh.setPower(0);
    c.idle();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  
}

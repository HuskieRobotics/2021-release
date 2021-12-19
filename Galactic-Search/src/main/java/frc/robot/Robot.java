/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoDriveDistance;
import frc.robot.commands.AutoTurnDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Timer timer;
  private boolean hasScheduledPath = false;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    this.hasScheduledPath = false;
    this.timer = new Timer();
    timer.reset();
    timer.start();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Time", timer.get());
    if (!this.m_robotContainer.getStorage().frontSensorUnblocked() && !this.hasScheduledPath) {
      if (this.timer.get()<100) {
        Command pathBlueA = new SequentialCommandGroup(
          new AutoTurnDistance(79, this.m_robotContainer.getDrivetrain()).withTimeout(1.5),
          new AutoDriveDistance(7.79, this.m_robotContainer.getDrivetrain()).withTimeout(2),
          new AutoTurnDistance(-5, this.m_robotContainer.getDrivetrain()).withTimeout(1),
          new AutoDriveDistance(12, this.m_robotContainer.getDrivetrain()).withTimeout(3)
        );
        pathBlueA.schedule();
        this.hasScheduledPath = true;
      }

      }

      /*
      // Change to 2.86 or greater
      if(this.timer.get() > 3) {
        Command pathRedA = new SequentialCommandGroup(
          new AutoDriveDistance(6, this.m_robotContainer.getDrivetrain()).withTimeout(1),
          new AutoTurnDistance(87, this.m_robotContainer.getDrivetrain()).withTimeout(.5),
          new AutoDriveDistance(11, this.m_robotContainer.getDrivetrain()).withTimeout(1.5),
          new AutoTurnDistance(40, this.m_robotContainer.getDrivetrain()).withTimeout(.5), 
          new AutoDriveDistance(26, this.m_robotContainer.getDrivetrain()).withTimeout(4)
        );
        pathRedA.schedule();
        this.hasScheduledPath = true;
      }
      else {
        Command pathRedB = new SequentialCommandGroup(
          new RunCommand(() -> this.m_robotContainer.getDrivetrain().setPowerAuto(0, 0, 1)).withTimeout(0.1),
          new AutoTurnDistance(-52, this.m_robotContainer.getDrivetrain()).withTimeout(0.5),
          new AutoDriveDistance(10, this.m_robotContainer.getDrivetrain()).withTimeout(1),
          new AutoTurnDistance(60.5, this.m_robotContainer.getDrivetrain()).withTimeout(1),
          new AutoDriveDistance(16, this.m_robotContainer.getDrivetrain()).withTimeout(5),
          new AutoTurnDistance(0, this.m_robotContainer.getDrivetrain()).withTimeout(1),
          new AutoDriveDistance(27, this.m_robotContainer.getDrivetrain()).withTimeout(5));
        pathRedB.schedule();
        this.hasScheduledPath = true;
      }*/
    }  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

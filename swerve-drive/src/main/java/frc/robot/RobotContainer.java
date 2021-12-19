/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SwerveSimCommand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.Constants.RobotConstants;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain;
  private final JoystickButton[] joystickButtons0;
  private final JoystickButton[] joystickButtons1;
  private final Joystick joystick0;
  private final Joystick joystick1;
  private Translation2d frontLeftLocation;
  private Translation2d frontRightLocation; 
  private Translation2d backLeftLocation;
  private Translation2d backRightLocation;
  private Translation2d center;


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    joystick0 = new Joystick(DriveConstants.JOYSTICK_0);
    joystick1 = new Joystick(DriveConstants.JOYSTICK_1);
    joystickButtons0 = new JoystickButton[DriveConstants.NUM_JOYSTICK_BUTTONS];
    joystickButtons1 = new JoystickButton[DriveConstants.NUM_JOYSTICK_BUTTONS];
    this.frontLeftLocation = new Translation2d(-RobotConstants.WIDTH/2, RobotConstants.LENGTH/2);
    this.frontRightLocation = new Translation2d(RobotConstants.WIDTH/2, RobotConstants.LENGTH/2);
    this.backLeftLocation = new Translation2d(-RobotConstants.WIDTH/2, -RobotConstants.LENGTH/2);
    this.backRightLocation = new Translation2d(RobotConstants.WIDTH/2, -RobotConstants.LENGTH/2);
    this.center = new Translation2d(0,0);
    
    for(int i = 1; i <= DriveConstants.NUM_JOYSTICK_BUTTONS; i++) {
      joystickButtons0[i-1] = new JoystickButton(joystick0, i);
      joystickButtons1[i-1] = new JoystickButton(joystick1, i);
    }

    // Configure the button bindings
    configureButtonBindings();

    // instantiate subsystems
    m_drivetrain = new Drivetrain();
    
    //initialize default commands
    // TODO Revert to default field relative
    // m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.drive(joystick1.getX(), joystick0.getY(), -1*joystick0.getRawAxis(5), true), m_drivetrain));
    m_drivetrain.setDefaultCommand(new RunCommand(() -> m_drivetrain.drive(joystick0.getX(), -1*joystick0.getY(), -1*joystick0.getRawAxis(5)), m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // drivetrain button bindings
    //joystickButtons0[DriveConstants.JOYSTICK_INVERT_BUTTON].whenPressed(new InstantCommand(() -> m_drivetrain.invertMotors()));
    //TODO: changed from .whileHeld to .whenHeld, untested change, revert back to .whileHeld if doesn't work
    //joystickButtons0[DriveConstants.JOYSTICK_HIGHGEAR_BUTTON].whenHeld(new RunCommand(() -> m_drivetrain.setGear(false)));
    //joystickButtons0[DriveConstants.JOYSTICK_HIGHGEAR_BUTTON].whenReleased(new RunCommand(() -> m_drivetrain.setGear(true)));
    joystickButtons0[DriveConstants.JOYSTICK_XSTANCE_BUTTON].whenHeld(new InstantCommand(() -> m_drivetrain.disableJoystickControls()));
    joystickButtons0[DriveConstants.JOYSTICK_XSTANCE_BUTTON].whileHeld(new RunCommand(() -> m_drivetrain.setXStance()));
    joystickButtons0[DriveConstants.JOYSTICK_XSTANCE_BUTTON].whenReleased(new InstantCommand(() -> m_drivetrain.enableJoystickControls()));
    joystickButtons0[DriveConstants.JOYSTICK_FIELD_RELATIVE].whenReleased(new InstantCommand(() -> m_drivetrain.disableFieldRelative()));
    joystickButtons0[DriveConstants.JOYSTICK_FIELD_RELATIVE].whenHeld(new InstantCommand(() -> m_drivetrain.enableFieldRelative()));
    joystickButtons0[8].whenHeld(new InstantCommand(() -> m_drivetrain.setCenterGrav(this.frontLeftLocation)));
    joystickButtons0[9].whenHeld(new InstantCommand(() -> m_drivetrain.setCenterGrav(this.frontRightLocation)));
    joystickButtons0[10].whenHeld(new InstantCommand(() -> m_drivetrain.setCenterGrav(this.backLeftLocation)));
    joystickButtons0[11].whenHeld(new InstantCommand(() -> m_drivetrain.setCenterGrav(this.backRightLocation)));
    joystickButtons0[2].whenHeld(new InstantCommand(() -> m_drivetrain.setCenterGrav(this.center)));
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // Replace null with auto command
    return new SwerveSimCommand();
  }
}

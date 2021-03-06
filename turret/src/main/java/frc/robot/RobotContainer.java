/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Turret;
import frc.robot.commands.TurnBaseToSetpoint;
import frc.robot.commands.TurnHoodToSetpoint;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Turret turret = new Turret();

  private final JoystickButton[] joystickButtons0;
  private final JoystickButton[] joystickButtons1;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    final Joystick joystick0 = new Joystick(0);
    final Joystick joystick1 = new Joystick(1);

    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];

    for(int i = 1; i <= joystickButtons0.length; i++) {
      joystickButtons0[i-1] = new JoystickButton(joystick0, i);
      joystickButtons1[i-1] = new JoystickButton(joystick1, i);
    }
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    joystickButtons0[2].whileHeld(new TurnBaseToSetpoint(turret, 0));
    joystickButtons0[2].whileHeld(new TurnHoodToSetpoint(turret, 0));

    // joystickButtons0[0].whileHeld(new RunCommand(() -> turret.setShooterPower(0.9)));
    joystickButtons0[0].whileHeld(new RunCommand(() -> turret.setShooterPower()));
    joystickButtons0[0].whenReleased(new RunCommand(() -> turret.setShooterPower(0.0)));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
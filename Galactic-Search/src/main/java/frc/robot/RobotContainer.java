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

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final JoystickButton[] joystickButtons0;
  private final JoystickButton[] joystickButtons1;
  private final Joystick joystick0;
  private final Joystick joystick1;
  private final Drivetrain drivetrain = new Drivetrain();
  SendableChooser<Command> chooser = new SendableChooser<>();

  private final DoubleTrigger shuttlingTrigger;
  private final DoubleTrigger shootingTrigger;
  private final DoubleTrigger toggleIntakeStateTrigger;
  private final Button[] operatorButtons;
  private final XboxController opeController;
  private final Collector collector = new Collector(); 
  private final Storage storage = new Storage ();
  private final Shooter shooter = new Shooter();

  private Command autoDoNothing = new RunCommand(() -> drivetrain.setPower(0, 0));
  private Command autoDriveAndTurn =
    new SequentialCommandGroup(
      new ParallelRaceGroup(
        new AutoDriveDistance(AutoConstants.DRIVE_STRAIGHT_DISTANCE, drivetrain),
        new WaitCommand(2)),
      new ParallelRaceGroup(
        new AutoTurnDistance(AutoConstants.DRIVE_90TURN_DISTANCE, drivetrain),
        new WaitCommand(1)),
      new ParallelRaceGroup(
        new AutoDriveDistance(10, drivetrain),
        new WaitCommand(2)),
      new ParallelRaceGroup(
        new AutoTurnDistance(0, drivetrain),
        new WaitCommand(1)),  
      new ParallelRaceGroup(
        new AutoDriveDistance(20, drivetrain),
        new WaitCommand(2)));

  private Command autoDriveStraight =
    new ParallelRaceGroup(
      new AutoDriveDistance(AutoConstants.DRIVE_STRAIGHT_DISTANCE, drivetrain),
      new WaitCommand(5)
    );
  private Command auto45TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_45TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );
  private Command auto90TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_90TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );  
  private Command autoNeg90TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_NEG90TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );  
  private Command auto180TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_180TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );  
  private Command auto270TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_270TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );
  private Command auto360TurnDistance =
    new ParallelRaceGroup(
      new AutoTurnDistance(AutoConstants.DRIVE_360TURN_DISTANCE, drivetrain),
      new WaitCommand(5)
    );  
  private Command autoLimeLightTurnToTarget = 
    new ParallelRaceGroup(
      new LimelightCenter(drivetrain),
      new WaitCommand(5)
    );

  private Command driveAndIntake = 
    new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetEncoders()),
      new ParallelCommandGroup(
        new InstantCommand(() -> drivetrain.setGear(false)),
        new InstantCommand(() -> toggleIntakeState()),
        new WaitCommand(2)),
      new ParallelRaceGroup(
        new AutoDriveDistance(13, drivetrain),
        new WaitCommand(15)
      )
    );
  
    



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    this.joystick0 = new Joystick(0);
    this.joystick1 = new Joystick(1);
    this.opeController = new XboxController(2);
    this.joystickButtons0 = new JoystickButton[13];
    this.joystickButtons1 = new JoystickButton[13];
    this.operatorButtons = new Button[17];
    for(int i = 1; i <= joystickButtons0.length; i++) {
        joystickButtons0[i-1] = new JoystickButton(joystick0, i);
        joystickButtons1[i-1] = new JoystickButton(joystick1, i);
        operatorButtons[i-1] = new JoystickButton(opeController, i);
    }
    operatorButtons[13] = new POVButton(opeController,0);
    operatorButtons[14] = new POVButton(opeController,180);
    operatorButtons[15] = new TriggerButton(opeController, true);
    operatorButtons[16] = new TriggerButton(opeController, false);


    shuttlingTrigger = new DoubleTrigger(joystickButtons1[0], joystickButtons1[3]);
    shootingTrigger = new DoubleTrigger(joystickButtons1[0], joystickButtons1[3].negate());
    toggleIntakeStateTrigger = new DoubleTrigger(joystickButtons1[2], joystickButtons1[0].negate());
    
    configureButtonBindings();

    // Configure the button bindings
    configureButtonBindings();

    chooser.addObject("Do Nothing", autoDoNothing);
    chooser.addObject("Drive Straight", autoDriveStraight);
    chooser.addObject("Turn 90", auto90TurnDistance);
    chooser.addObject("Turn Negative 90", autoNeg90TurnDistance);
    chooser.addObject("Turn 180", auto180TurnDistance);
    chooser.addObject("Turn 180",auto180TurnDistance);
    chooser.addObject("Drive and Pick Up", driveAndIntake);
    // chooser.setDefaultOption("Drive Straight", autoDriveStraight);
    chooser.setDefaultOption("Drive and Pick Up", driveAndIntake);

    // instantiate subsystems
    
    //initialize default commands
    drivetrain.setDefaultCommand(new RunCommand(() -> drivetrain.arcadeDrive(0.3*joystick1.getX(), -1*joystick0.getY()), drivetrain));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drivetrain button bindings
    joystickButtons0[DriveConstants.JOYSTICK_INVERT_BUTTON].whenPressed(new InstantCommand(() -> drivetrain.invertMotors()));
    //TODO: changed from .whileHeld to .whenHeld, untested change, revert back to .whileHeld if doesn't work
    joystickButtons0[DriveConstants.JOYSTICK_HIGHGEAR_BUTTON].whenHeld(new RunCommand(() -> drivetrain.setGear(false)));
    joystickButtons0[DriveConstants.JOYSTICK_HIGHGEAR_BUTTON].whenReleased(new RunCommand(() -> drivetrain.setGear(true)));
    operatorButtons[7].whenHeld(new RunCommand(() -> storage.setPower(StorageConstants.manualStorageSpeed), storage));
    operatorButtons[6].whenHeld(new RunCommand(() -> storage.setPower(-StorageConstants.manualStorageSpeed), storage));
    operatorButtons[5].whenHeld(new RunCommand(() -> shooter.setPower(ShooterConstants.manualShooterSpeed), shooter));
    operatorButtons[2].whenHeld(new RunCommand(() -> collector.setPower(CollectorConstants.manualIntakeSpeed), collector));
    operatorButtons[0].whenPressed(new InstantCommand(() -> collector.deployIntake(!collector.collectorOut()), collector));
    joystickButtons0[2].whenHeld(new LimelightCenter(drivetrain));
    joystickButtons0[1].whileHeld(new LimelightSetPower(collector, storage, shooter, StorageConstants.storageFastSpeed));

    shuttlingTrigger.whileHeld(
      new Shoot(collector, storage, shooter, 
                ShooterConstants.shuttlingSpeed, StorageConstants.storageSlowSpeed)
    );
    shootingTrigger.whileHeld(
      new Shoot(collector, storage, shooter, 
                ShooterConstants.shootingSpeed, StorageConstants.storageFastSpeed)
      // new LimelightSetPower(collector, storage, shooter, StorageConstants.storageSlowSpeed)
    );
    toggleIntakeStateTrigger.
    whenPressed(new InstantCommand(() -> toggleIntakeState()));
  }

  public void toggleIntakeState () {
    SmartDashboard.putString("toggle state", (RobotGlobal.state));
 
    if (RobotGlobal.state.equals("Idle") && (storage.frontSensorUnblocked() || storage.backSensorUnblocked())) {
      SmartDashboard.putNumber("intake state counter", SmartDashboard.getNumber("intake state counter", 0)+1);
      CommandScheduler.getInstance().schedule(new Intake(collector, storage));
      //RobotGlobal.state = "Intake";
    }
    else{
      SmartDashboard.putNumber("idle state counter", SmartDashboard.getNumber("idle state counter", 0)+1);
      storage.setPower(0);
      shooter.setPower(0);
      collector.idle();
      RobotGlobal.state = "Idle";
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public Storage getStorage() {
    return this.storage;
  }

  public Drivetrain getDrivetrain() {
    return this.drivetrain;
  }
}

class DoubleTrigger extends Button{
  private Trigger trigger1;
  private Trigger trigger2;

  public DoubleTrigger(Trigger t1, Trigger t2){
      trigger1 = t1;
      trigger2 = t2;
  }
  
  public boolean get(){
      return trigger1.get() && trigger2.get();
  }
}
class TriggerButton extends Button {
  private XboxController joystick;
  private boolean left;
  public TriggerButton (XboxController j, boolean l) {
    joystick = j;
    left = l;
  }
  public boolean get() {
    if (left) return joystick.getTriggerAxis(Hand.kLeft) > .8;
    return joystick.getTriggerAxis(Hand.kRight) > .8;
  }
}

class POVButton extends Button{
  private XboxController joystick;
  private int desiredValue;

  public POVButton(XboxController j, int triggerNum)
  {
    this.joystick = j;
    this.desiredValue = triggerNum;
  }

  public boolean get() {
    if (desiredValue == 0)
      return this.joystick.getPOV() == this.desiredValue || 
             this.joystick.getPOV() == 315 || this.joystick.getPOV() == 45;
    return this.joystick.getPOV() == this.desiredValue || 
           this.joystick.getPOV() == 135 || this.joystick.getPOV() == 225;
  }
}



class DoubleButton extends Button{
  private Button button1;
  private Button button2;
  private boolean button1getTrue;
  private boolean button2getTrue;

  public DoubleButton(Button b1, Button b2, boolean b1getTrue, boolean b2getTrue){
      button1 = b1;
      button2 = b2;
      button1getTrue = b1getTrue;
      button2getTrue = b2getTrue;
  }
  
  public boolean get(){
      boolean b1 = button1.get();
      boolean b2 = button2.get();
      if(!button1getTrue) b1 = !b1;
      if(!button2getTrue) b2 = !b2;

      return b1 && b2;
  }
}

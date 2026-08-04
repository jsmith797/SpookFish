// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.swervedrive.auto.Autos;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(OperatorConstants.SCALE_TRANSLATION)
                                                            .scaleRotation(OperatorConstants.SCALE_ROTATION)
                                                            .allianceRelativeControl(true);

   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()  // Copy the stream so further changes do not affect driveAngularVelocity
                                                            .withControllerHeadingAxis(() -> driverXbox.getRightX() * -1,
                                                                                       () -> driverXbox.getRightY() * -1) // Axis which give the desired heading angle using trigonometry.
                                                            .headingWhile(true); // Enable heading based control.

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // register commands for pathplanner
    registerCommands();

    // Configure the trigger bindings
    configureBindings();
    // DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    // autoChooser.setDefaultOption("Do Nothing", Commands.none());

    //Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);  
  }

  // helper method to register commands for pathplanner
private void registerCommands()
{
  NamedCommands.registerCommand("test command" , drivebase.testCommand());
}

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // main teleop swerve drive command
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    // press B to reset gyro
    driverXbox.b().onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));

    // press A to perform the test command
    driverXbox.a().onTrue(drivebase.testCommand());

    // press right stick to set rotation speed to 1 rotation per second
    driverXbox.rightStick().onTrue(Commands.runOnce(() -> driveAngularVelocity.scaleRotation(1)));

    // release right stick to set rotation speed back
    driverXbox.rightStick().onFalse(Commands.runOnce(() -> driveAngularVelocity.scaleRotation(Constants.OperatorConstants.SCALE_ROTATION)));
  
    // press to print max angular velocity
    driverXbox.x().onTrue(Commands.runOnce(()->System.out.println(drivebase.getMaximumChassisAngularVelocity())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

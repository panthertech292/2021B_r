// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final static XboxController io_drivercontroller = new XboxController(Constants.kDriverController);
  private final static XboxController io_opercontroller = new XboxController(Constants.kOperController);
  private double v_AutoDistance;
  private double v_AutoAngle;
  private double v_Time;
  private double v_LeftSpeed;
  private double v_RightSpeed;
  private double v_TargetRPM;
  private double v_TargetSpeed;
  final static int io_lefttrigger = 2;
  final static int io_righttrigger = 3;

  // Subsystems
  private final DriveSubsystem s_DriveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem s_ShooterSubsystem = new ShooterSubsystem();
  private final BeltSubsystem s_BeltSubsystem = new BeltSubsystem();

  // Auto Commands
  private final Command z_AutoForward = new AutoForward(s_DriveSubsystem, v_LeftSpeed, v_RightSpeed, v_Time);
  private final Command z_AutoForwardPID = new AutoForwardPID(s_DriveSubsystem, v_LeftSpeed, v_RightSpeed, v_AutoDistance);
  private final Command z_AutoForwardEncoder = new AutoForwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoBackwardEncoder = new AutoBackwardEncoder(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoBackward = new AutoBackward(s_DriveSubsystem);
  private final Command z_AutoRight90Encoder = new AutoRight90Encoder(s_DriveSubsystem);
  private final Command z_AutoRight90Timed = new AutoRight90Encoder(s_DriveSubsystem);
  private final Command z_AutoRight90Gyro = new AutoRight90Gyro(s_DriveSubsystem, v_AutoAngle, v_LeftSpeed,
      v_RightSpeed);
  private final Command z_AutoDead = new AutoDead(s_DriveSubsystem);
  private final Command z_AutoBounce = new AutoBounce(s_DriveSubsystem);
  private final Command z_AutoSquareRight = new AutoSquareRight(s_DriveSubsystem);

  private final Command z_AutoShootTimed = new AutoShootTimed(s_ShooterSubsystem);
  private final Command z_AutoBarrel = new AutoBarrel(s_DriveSubsystem);

  private final Command z_AutoDriveVisionCorrection = new AutoDriveVisionCorrection(s_DriveSubsystem, v_AutoDistance, v_LeftSpeed, v_RightSpeed);

  // Vision Commands
  private final Command z_VisionAlign = new VisionAlign(s_DriveSubsystem);
  private final Command z_VisionDistance = new VisionDistance(s_DriveSubsystem);
  private final Command z_VisionAll = new VisionAll(s_DriveSubsystem);
  // Drive Commands
  private final Command z_DriveTeleop = new DriveTeleop(s_DriveSubsystem);

  // Shooter Commands
  private final Command z_ShooterFireHalf = new ShooterFireHalf(s_ShooterSubsystem);
  private final Command z_ShooterFireFull = new ShooterFireFull(s_ShooterSubsystem);
  private final Command z_AimAdjustDown = new AimAdjustDown(s_ShooterSubsystem);
  private final Command z_AimAdjustUp = new AimAdjustUp(s_ShooterSubsystem);

  //Belt Commands
  private final Command z_BeltForwardAll = new BeltForwardAll(s_BeltSubsystem);

  SendableChooser<Command> o_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    o_chooser.addOption("Auto Forward", z_AutoForward);
    o_chooser.addOption("Auto Forward Encoder", z_AutoForwardEncoder);
    o_chooser.addOption("Auto Backward Encoder", z_AutoBackwardEncoder);
    o_chooser.addOption("Auto Backward", z_AutoBackward);
    o_chooser.addOption("Auto RIght 90 Encoder", z_AutoRight90Encoder);
    o_chooser.addOption("Auto Right 90 Timed", z_AutoRight90Timed);
    o_chooser.addOption("Auto Right 90 Gyro", z_AutoRight90Gyro);

    // Vision stuff
    o_chooser.addOption("Vision Right", z_VisionAlign);
    Shuffleboard.getTab("Autonomous").add(o_chooser);
    s_DriveSubsystem.setDefaultCommand(z_DriveTeleop);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Driver
    final JoystickButton d_aButton = new JoystickButton(io_drivercontroller, Button.kA.value);
    final JoystickButton d_bButton = new JoystickButton(io_drivercontroller, Button.kB.value);
    final JoystickButton d_xButton = new JoystickButton(io_drivercontroller, Button.kX.value);
    final JoystickButton d_startButton = new JoystickButton(io_drivercontroller, Button.kStart.value);
    final JoystickButton d_backButton = new JoystickButton(io_drivercontroller, Button.kBack.value);
    final JoystickButton d_yButton = new JoystickButton(io_drivercontroller, Button.kY.value);
   
    //Operator
    final JoystickButton o_aButton = new JoystickButton(io_opercontroller, Button.kA.value);
    final JoystickButton o_bButton = new JoystickButton(io_opercontroller, Button.kB.value);
    final JoystickButton o_xButton = new JoystickButton(io_opercontroller, Button.kX.value);
    final JoystickButton o_yButton = new JoystickButton(io_opercontroller, Button.kY.value);

    // Driver Button Binds
    d_aButton.whileHeld(z_BeltForwardAll);
    d_bButton.whenPressed(z_AutoBarrel);
    d_xButton.whenPressed(z_ShooterFireFull);
    d_yButton.whileHeld(z_ShooterFireHalf);
    d_backButton.whileHeld(z_AimAdjustDown);
    d_startButton.whileHeld(z_AimAdjustUp);

    //Operator Button Binds
    o_aButton.whileHeld(z_AutoBarrel);
    o_yButton.whenPressed(z_AutoRight90Encoder);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public static double getLeftSpeed() {
    return io_drivercontroller.getY(GenericHID.Hand.kLeft);
  }

  public static double getRightSpeed() {
    return io_drivercontroller.getY(GenericHID.Hand.kRight);
  }
  public Command getAutonomousCommand() {
    //An ExampleCommand will run in autonomous
   return o_chooser.getSelected();
 }
}

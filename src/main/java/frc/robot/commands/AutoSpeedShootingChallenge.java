// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSpeedShootingChallenge extends SequentialCommandGroup {
  /** Creates a new AutoSpeedShootingChallenge. */
  private final DriveSubsystem DriveSubsystem;
  private final ShooterSubsystem ShooterSubsystem;

  public AutoSpeedShootingChallenge(DriveSubsystem s_DriveSubsystem, ShooterSubsystem s_ShooterSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    ShooterSubsystem = s_ShooterSubsystem;


   addRequirements(s_DriveSubsystem, s_ShooterSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}

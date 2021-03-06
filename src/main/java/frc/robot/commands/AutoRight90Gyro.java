/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class AutoRight90Gyro extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private double AutoAngle;
  private double LeftSpeed;
  private double RightSpeed;
  /**
   * Creates a new AutoRight90Gyro.
   */
  public AutoRight90Gyro(DriveSubsystem s_DriveSubsystem, double v_AutoAngle, double v_LeftSpeed, double v_RightSpeed) {
    DriveSubsystem = s_DriveSubsystem;
    AutoAngle = v_AutoAngle;
    LeftSpeed = v_LeftSpeed;
    RightSpeed = v_RightSpeed;
    addRequirements(s_DriveSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.zeroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(LeftSpeed,RightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  DriveSubsystem.gyroFinish(AutoAngle);
  }
}

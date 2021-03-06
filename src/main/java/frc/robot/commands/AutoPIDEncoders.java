/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPIDEncoders extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  private final double TargetSpeedLeft;
  private final double TargetSpeedRight;
  private double v_Angle;
  private double v_PIDRatio;
  

  /**
   * Creates a new AutoForward.
   */
  public AutoPIDEncoders(DriveSubsystem s_DriveSubsystem, double v_TargetSpeedLeft, double v_TargetSpeedRight, double v_Angle, double v_PIDRatio) {
    DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);

    
    TargetSpeedLeft = v_TargetSpeedLeft;
    TargetSpeedRight = v_TargetSpeedRight;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.resetTimer();
    DriveSubsystem.zeroDistanceSensors();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.changePowerSetPoints(v_PIDRatio*DriveSubsystem.RightPID(TargetSpeedLeft),DriveSubsystem.LeftPID(TargetSpeedRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    DriveSubsystem.changePowerSetPoints(0,0);
    System.out.println(DriveSubsystem.getLeftPosition());
    System.out.println(DriveSubsystem.getRightPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.gyroFinish(v_Angle);
  }
}

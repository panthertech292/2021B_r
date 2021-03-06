/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class VisionAlign extends CommandBase {
  private final DriveSubsystem DriveSubsystem;
  /**
   * Creates a new VisionAlign.
   */

  public VisionAlign(DriveSubsystem s_DriveSubsystem) {
    DriveSubsystem = s_DriveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_DriveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.changePowerSetPoints(0,0);
    DriveSubsystem.zeroDistanceSensors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Running the move command");
    DriveSubsystem.driveModePowerSetPoint();
    DriveSubsystem.visionAlignLeft();
    DriveSubsystem.visionAlignRight();
    
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.changePowerSetPoints(0,0);
    System.out.println("Left Encoder Value"  +   DriveSubsystem.getLeftPosition());
    System.out.println("Right Encoder Value"  +   DriveSubsystem.getRightPosition());

    System.out.println("Perceived Angle"   +  DriveSubsystem.PerceivedAngle(DriveSubsystem.getLeftPosition()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DriveSubsystem.visionFinish() ;
  }
}

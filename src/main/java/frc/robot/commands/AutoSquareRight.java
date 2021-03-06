/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoSquareRight extends SequentialCommandGroup {
  /**
   * Creates a new AutoSquareRight.
   */
  private final DriveSubsystem DriveSubsystem;
  public AutoSquareRight(DriveSubsystem s_DriveSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
   // super();
   DriveSubsystem = s_DriveSubsystem;

   addRequirements(s_DriveSubsystem);

   addCommands(
//new AutoForwardEncoder(s_DriveSubsystem, 24.0, 1.0, 1.0),
    //new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
    //new AutoRight90Gyro(s_DriveSubsystem, 87, 0.5, -.25),
    //new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(75), 1.0, -.35),
    //new AutoForward(s_DriveSubsystem, 10000000, -0.6, 0.6)

    //new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(103), 1.0, -.25)
    //new AutoDriveWithVisionCorrection(s_DriveSubsystem, 3.0, 0.55)
    
   // new VisionAlign(s_DriveSubsystem)
    //new AutoTurnGyroWithVisionFinish(s_DriveSubsystem, 90.0, .7, .2),
    //new AutoDriveVisionCorrection(s_DriveSubsystem, 24.0, .65, .65)
    new AutoRight90Gyro(s_DriveSubsystem, 140, 0.65, -.65) //Gives 180 Degrees
  //new AutoTurnPID(s_DriveSubsystem, .85, .00, 78.99704244)
//new AutoBarrel(s_DriveSubsystem)

   /* new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
   new AutoRight90Gyro(s_DriveSubsystem, 87, 0.5, -.25),
    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(81.3), 1.0, -.25),
    new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
    new AutoForwardEncoder(s_DriveSubsystem, 24.0, 0.5, 0.5),
    new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
   // new AutoRight90Gyro(s_DriveSubsystem, 87, 0.5, -.25),
    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(81.3), 1.0, -.25),
    new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
    new AutoForwardEncoder(s_DriveSubsystem, 24.0, 0.5, 0.5),
    new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0),
    new AutoForwardEncoder(s_DriveSubsystem, DriveSubsystem.rotateRobot(81.3), 1.0, -.25),
    //new AutoRight90Gyro(s_DriveSubsystem, 87, 0.5, -.25),
    new AutoForward(s_DriveSubsystem, 0.25, 0.0, 0.0) */

    

   );
  }
}

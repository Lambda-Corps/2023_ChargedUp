// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroPID;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.ScoreCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos1ScoreMovePickupPiece extends SequentialCommandGroup {
  /** Creates a new Pos1ScoreMovePickupPiece. */
  public Pos1ScoreMovePickupPiece(DriveTrain dt, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreCone(gripper),
      new DriveMotionMagic(dt, -148),
      new TurnToAngleWithGyroPID(dt, 90),
      new DriveMotionMagic(dt, 65)
    );
  }

  // private void addCommands(ScoreCone scoreCone, DriveMotionMagic driveMotionMagic) {
  // }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.StowArmManually;
import frc.robot.subsystems.Arm.WristDriveToPositionPIDTest;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommandTest;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.ScoreCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos1ScoreMove extends SequentialCommandGroup {
  Arm m_arm;
  Gripper m_gripper;
  DriveTrain m_dt;
  public Pos1ScoreMove(DriveTrain dt, Gripper gripper, Arm arm) {
    m_arm = arm;
    m_gripper = gripper;
    m_dt = dt;

    addCommands(
      new WristThenArmSequenceCommandTest(arm, SuperStructurePosition.ScoreConeMid).raceWith(new WaitCommand(4)),
      new WaitCommand(0.3),
      gripper.expandGripperCommand(),
      new StowArmManually(arm),
      new WristDriveToPositionPIDTest(arm, SuperStructurePosition.Stowed).raceWith(new WaitCommand(3)),
      new DriveMotionMagic(dt, -148)
    );
  }
}

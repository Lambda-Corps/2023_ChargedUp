// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.DriveTrain.BalanceBangBangCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveSlowlyUntilRamp;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroPID;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.ScoreCone;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos1ScoreLowMoveBalance extends SequentialCommandGroup {
  /** Creates a new ReplaceMeSequentialCommandGroup. */
  public Pos1ScoreLowMoveBalance(DriveTrain dt, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new WristThenArmSequenceCommand(arm, SuperStructurePosition.ScoreLow).raceWith(new WaitCommand(3)),
      // new MoveWristToPositionMM(arm, SuperStructurePosition.ScoreLow),
      // new PrintCommand("Wrist the arm done"),
      // Drop the cone on the peg
      // gripper.expandGripperCommand(),
      new ScoreCone(gripper),
       // new PrintCommand("Expand Gripper done"),
      // Stow the arm back in the robot
      // new StowSuperStructure(arm),
      //Drive Back 100 inches 
      new DriveMotionMagic(dt, -148), 
      new TurnToAngleWithGyroPID(dt, 90),
      new DriveMotionMagic(dt, 65),
      new TurnToAngleWithGyroPID(dt, -90),
      // This needs to be drive and Bang Bang
      new DriveSlowlyUntilRamp(dt).raceWith(new WaitCommand(1.25)), 
      new BalanceBangBangCommand(dt)
    );
  }
}

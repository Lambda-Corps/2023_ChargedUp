// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.StowSuperStructure;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.Arm.ArmSuperStructurePosition;
import frc.robot.subsystems.DriveTrain.BalanceBangBangCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveSlowlyUntilRamp;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.Wrist.WristSuperStructurePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos2ScoreHighMobilityBalance extends SequentialCommandGroup {
  /** Creates a new Pos2ScoreMidMobilityBalance. */
  public Pos2ScoreHighMobilityBalance(DriveTrain drivetrain, Arm arm, Wrist wrist, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristThenArmSequenceCommand(arm, wrist, ArmSuperStructurePosition.ScoreConeHigh, WristSuperStructurePosition.ScoreConeHigh),
      gripper.expandGripperCommand(),
      new StowSuperStructure(arm, wrist),
      new DriveMotionMagic(drivetrain, -150, true),
      new DriveSlowlyUntilRamp(drivetrain).withTimeout(1.5),
      new BalanceBangBangCommand(drivetrain)
    );
  }
}

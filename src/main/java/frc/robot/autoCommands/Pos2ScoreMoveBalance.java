// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import frc.robot.subsystems.DriveTrain.BalanceBangBangCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveSlowlyUntilRamp;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.StowSuperStructure;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroPID;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.RunMotorsBackward;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos2ScoreMoveBalance extends SequentialCommandGroup {
  /** Creates a new Pos2ScoreMoveBalance. */
  public Pos2ScoreMoveBalance(DriveTrain dt, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new WristThenArmSequenceCommand(arm, SuperStructurePosition.ScoreConeMid).raceWith(new WaitCommand(3)),
      // // new PrintCommand("Wrist the arm done"),
      // // Drop the cone on the peg
      // gripper.expandGripperCommand(),
      // // Stow the arm back in the robot
      // new StowSuperStructure(arm),
      new RunMotorsBackward(gripper).withTimeout(1),
      //Drive Back 100 inches 
      new DriveMotionMagic(dt, -140),  
      // This needs to be drive and Bang Bang
      new DriveSlowlyUntilRamp(dt).raceWith(new WaitCommand(1.5)), 
      new BalanceBangBangCommand(dt),
      new TurnToAngleWithGyroPID(dt, 90)
    );
  }
}

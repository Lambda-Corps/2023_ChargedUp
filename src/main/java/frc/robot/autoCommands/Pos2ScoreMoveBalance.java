// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import frc.robot.subsystems.DriveTrain.BalanceBangBangCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.DriveTrain.DriveSlowlyUntilRamp;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmThenWristSequenceCommand;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.Gripper.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Pos2ScoreMoveBalance extends SequentialCommandGroup {
  /** Creates a new Pos2ScoreMoveBalance. */
  public Pos2ScoreMoveBalance(DriveTrain dt, Arm arm, Gripper gripper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /* new WristThenArmSequenceCommand(arm, SuperStructurePosition.ScoreConeMid).raceWith(new WaitCommand(3)), */
      new WaitCommand(3),
      // TODO test these new commands and remove the prints
      new PrintCommand("Wrist the arm done"),
      // Drop the cone on the peg
      /*gripper.expandGripperCommand(),*/
      new WaitCommand(1),
      new PrintCommand("Expand Gripper done"),
      // Stow the arm back in the robot
      /*new ArmThenWristSequenceCommand(arm, SuperStructurePosition.Stowed).raceWith(new WaitCommand(3)), */
      new WaitCommand(3),
      new PrintCommand("ArmThenWrist Done"),
      //Drive Back 100 inches 
      new DriveMotionMagic(dt, -140), 
     
      // This needs to be drive and Bang Bang
      new DriveSlowlyUntilRamp(dt), 
      new BalanceBangBangCommand(dt)
    );
  }
}
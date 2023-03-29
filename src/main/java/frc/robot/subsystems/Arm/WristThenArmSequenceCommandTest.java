// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.Wrist.WristSuperStructurePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristThenArmSequenceCommandTest extends SequentialCommandGroup {
  /** Creates a new WristThenArmSequenceCommand. */
  public WristThenArmSequenceCommandTest(Arm arm_sub, Wrist wrist, Arm.ArmSuperStructurePosition position_req) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      arm_sub.set_state(ArmState.Moving),
      new WristDriveToPositionPIDTest(arm_sub, wrist, position_req, WristSuperStructurePosition.Manual),
      new ArmDriveToPositionPIDTest(arm_sub, wrist, position_req),
      arm_sub.set_state(ArmState.Holding)
    );
  }
}

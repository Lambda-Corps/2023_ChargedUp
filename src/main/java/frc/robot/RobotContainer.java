// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autoCommands.PosTwoPreloadMobilBalance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmDriveToPositionPIDTest;
import frc.robot.subsystems.Arm.ArmThenWristSequenceCommand;
import frc.robot.subsystems.Arm.DriveArmManually;
import frc.robot.subsystems.Arm.WristDriveToPositionPIDTest;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.DriveTrain.BalanceBangBangCommandTest;
import frc.robot.subsystems.DriveTrain.DefaultDriveTrainCommand;
import frc.robot.subsystems.DriveTrain.DriveMMSequenceTest;
import frc.robot.subsystems.DriveTrain.DriveMotionMagicTest;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.FineGrainedDrivingControl;
import frc.robot.subsystems.DriveTrain.SetMaxSpeedCommand;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroTest;
import frc.robot.subsystems.Gripper.Gripper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Human interface
  private final CommandXboxController m_driver_controller = new CommandXboxController(0);
  private final CommandXboxController m_partner_controller = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_drivetrain = new DriveTrain();
  private final Arm m_arm = new  Arm();
  private final Gripper m_gripper = new  Gripper();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    buildDriveTestTab();
    // buildArmTestTab();
    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(new DefaultDriveTrainCommand(m_drivetrain, m_driver_controller));    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_partner_controller.rightBumper().whileTrue(new DriveArmManually(m_arm, m_partner_controller));
    m_partner_controller.start().onTrue(m_arm.stopArmAndWristCommand());
    m_partner_controller.leftStick().onTrue(m_gripper.contractGripperCommand());
    m_partner_controller.rightStick().onTrue(m_gripper.expandGripperCommand());
    
    m_driver_controller.leftBumper().onTrue(m_drivetrain.shiftToHighGear());
    m_driver_controller.leftBumper().onFalse(m_drivetrain.shiftToLowGear());

    m_partner_controller.x().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeMid),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeMid),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.ScoreCubeMid)));
    m_partner_controller.y().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeHigh),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeHigh),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.ScoreCubeHigh)));
    m_partner_controller.a().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreConeMid),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreConeMid),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.ScoreConeMid)));
    m_partner_controller.b().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreConeHigh),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreConeHigh),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.ScoreConeHigh)));
    m_partner_controller.povDown().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.GroundPickup),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.GroundPickup),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.GroundPickup)));
    m_partner_controller.povUp().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.SubstationPickup),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.SubstationPickup),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.SubstationPickup)));
    m_partner_controller.povLeft().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreLow),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreLow),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.ScoreLow)));
    m_partner_controller.povRight().onTrue(new ConditionalCommand(new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.Stowed),
                                                           new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.Stowed),
                                                           ()->m_arm.isBackwardMovement(SuperStructurePosition.Stowed)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto Needs to be Fixed");
  }

  private void buildDriveTestTab() {
    ShuffleboardTab driveTestTab = Shuffleboard.getTab("Drive Test");

    driveTestTab.add("Right Encoder", 0).withPosition(1, 0).withSize(1, 1);
    driveTestTab.add("Left Encoder", 0).withPosition(0, 0).withSize(1, 1);
    driveTestTab.add("Right Speed", 0).withPosition(3, 0).withSize(1,1);
    driveTestTab.add("Left Speed", 0).withPosition(2, 0).withSize(1,1);
    
    // Set the max speed variables
    driveTestTab.add("Max Speed", 0).withPosition(0, 1).withSize(1,1);
    // driveTestTab.add("Reset Max Speed", m_drivetrain.setMaxValue()).withPosition(1, 2).withSize(2, 1);
    // Set the max speed variables
    driveTestTab.add("Set_Max Speed", new SetMaxSpeedCommand(m_drivetrain)).withPosition(1, 1).withSize(2, 1);

    // Motion Magic
    driveTestTab.add("Target Distance", 0).withPosition(1, 3).withSize(1, 1);
    driveTestTab.add("Time to Velo", 0).withPosition(0, 3).withSize(1, 1);
    driveTestTab.add("Target Velocity", 0).withPosition(2, 3).withSize(1,1);
    driveTestTab.add("Left Encoder Result", 0).withPosition(3, 3).withSize(1,1);
    driveTestTab.add("Right Encoder Result", 0).withPosition(4, 3).withSize(1,1);
    driveTestTab.add("MM kP", 0).withPosition(5, 3).withSize(1,1);
    driveTestTab.add("Drive MM", new DriveMotionMagicTest(m_drivetrain)).withPosition(6,3).withSize(2, 1);
    driveTestTab.addDouble("Left Error", m_drivetrain::getLeftError).withPosition(0,4).withSize(1,1);
    driveTestTab.addDouble("Right Error", m_drivetrain::getRightError).withPosition(1,4).withSize(1,1);
    driveTestTab.add("DriveMM Sequence", new DriveMMSequenceTest(m_drivetrain)).withPosition(2, 4).withSize(1, 1);
    driveTestTab.add("Target Degrees", 0).withPosition(6, 1).withSize(1, 1);
    driveTestTab.addNumber("Curr Heading", m_drivetrain::getScaledHeading).withPosition(4,0).withSize(1,1);
    driveTestTab.add("Gyro", m_drivetrain.getGyro()).withPosition(3, 1).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    driveTestTab.addNumber("Yaw", m_drivetrain::getYaw).withPosition(0, 2).withSize(1, 1);
    driveTestTab.addNumber("Pitch", m_drivetrain::getPitch).withPosition(1, 2).withSize(1, 1);
    driveTestTab.addNumber("Roll", m_drivetrain::getRoll).withPosition(2, 2).withSize(1, 1);
    driveTestTab.addNumber("Angle", m_drivetrain::getAngle).withPosition(5, 2).withSize(1, 1);

    // test auto secquence
    driveTestTab.add("Pos 2 auto sequence", new PosTwoPreloadMobilBalance(m_drivetrain, m_arm, m_gripper));


    // PID Tuning
    driveTestTab.add("Turn PID", m_drivetrain.get_dt_turn_pidcontroller()).withPosition(5, 0);
    driveTestTab.add("Turn with PID", new TurnToAngleWithGyroTest(m_drivetrain)).withPosition(6, 0).withSize(2, 1);
    driveTestTab.add("Drive Fine Grained", new FineGrainedDrivingControl(m_drivetrain, m_driver_controller)).withPosition(8, 1).withSize(2, 1);
    
  }

  private void buildArmTestTab() {
    ShuffleboardTab armTestTab = Shuffleboard.getTab("Arm Test");

    armTestTab.add("ArmEncoder", 0).withPosition(0, 0).withSize(1, 1);
    armTestTab.addBoolean("ArmForward", m_arm::getArmForwardLimit).withPosition(1, 0).withSize(1,1);
    armTestTab.addBoolean("ArmReverse", m_arm::getArmReverseLimit).withPosition(2, 0).withSize(1,1);
    armTestTab.add("Arm Rev", 0).withPosition(3, 0).withSize(1, 1);
    
  
    armTestTab.add("WristEncoder", 0).withPosition(0, 1).withSize(1, 1);

    armTestTab.addBoolean("WristForward", m_arm::getWristForwardLimit).withPosition(1, 1).withSize(1,1);
    armTestTab.addBoolean("WristReverse", m_arm::getWristReverseLimit).withPosition(2, 1).withSize(1,1);
    armTestTab.add("Wrist Rev", 0).withPosition(3, 1).withSize(1, 1);
    armTestTab.add("Arm", m_arm).withPosition(8, 0).withSize(2, 1);
    armTestTab.add("Gripper", m_gripper).withPosition(8, 1).withSize(2, 1);
    armTestTab.add("Drivetrain", m_drivetrain).withPosition(8, 2).withSize(2, 1);
    
    // PIDF Values
    armTestTab.add("Arm kP", 0).withPosition(4, 0).withSize(1, 1);
    armTestTab.add("Arm kI", 0).withPosition(5, 0).withSize(1, 1);
    armTestTab.add("Arm kD", 0).withPosition(6, 0).withSize(1, 1);
    armTestTab.add("Arm kF", 0).withPosition(7, 0).withSize(1, 1);

    armTestTab.add("Wrist kP", 0).withPosition(4, 1).withSize(1, 1);
    armTestTab.add("Wrist kI", 0).withPosition(5, 1).withSize(1, 1);
    armTestTab.add("Wrist kD", 0).withPosition(6, 1).withSize(1, 1);
    armTestTab.add("Wrist kF", 0).withPosition(7, 1).withSize(1, 1);

    // armTestTab.add("Wrist Max", 0).withPosition(4, 2).withSize(1, 1);
    // armTestTab.add("Wrist Stator", 0).withPosition(5, 2).withSize(1, 1);
    // armTestTab.add("Arm Max", 0).withPosition(6, 2).withSize(1, 1);
    // armTestTab.add("Arm Stator", 0).withPosition(7, 2).withSize(1, 1);

    // armTestTab.add("Wrist Fwd Spd", .2).withPosition(4, 3).withSize(1, 1);
    // armTestTab.add("Wrist Rev Spd", .2).withPosition(5, 3).withSize(1, 1);
    // armTestTab.add("Arm Fwd Spd", .2).withPosition(6, 3).withSize(1, 1);
    // armTestTab.add("Arm Rev Spd", .2).withPosition(7, 3).withSize(1, 1);

    // armTestTab.add("Wrist Fwd Lim", 264000).withPosition(4, 2).withSize(1, 1);
    // armTestTab.add("Wrist Rev Lim", 32000).withPosition(5, 2).withSize(1, 1);
    // armTestTab.add("Arm Fwd Lim", 54613).withPosition(6, 2).withSize(1, 1);
    // armTestTab.add("Arm Rev Lim", 10000).withPosition(7, 2).withSize(1, 1);

    // Add the commands to the page
    armTestTab.add("Zero Wrist Encoder", m_arm.setWristEncoderToZero()).withPosition(0, 2).withSize(2, 1);
    armTestTab.add("Zero Arm Encoder", m_arm.setArmEncoderToZero()).withPosition(2, 2).withSize(2, 1);
    armTestTab.add("Time to Velo", 1).withPosition(4, 2).withSize(1, 1);
    armTestTab.add("Target Velocity", 1000).withPosition(5, 2).withSize(1, 1);
    armTestTab.add("Arm MM Error", 0).withPosition(6, 2).withSize(1, 1);
    armTestTab.add("Wrist MM Error", 0).withPosition(7, 2).withSize(1, 1);

    // armTestTab.add("Set Arm Max Speed", m_arm.setArmMaxSpeed()).withPosition(0, 3).withSize(2, 1);
    // armTestTab.add("Set Wrist Max Speed", m_arm.setWristMaxSpeed()).withPosition(2, 3).withSize(2, 1);

    // Add some test commands for the arm state machine
    // armTestTab.add("Stow Superstructure", m_arm.requestMoveArmCommand(SuperStructurePosition.Stowed).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.Stowed))).withPosition(0, 4).withSize(2, 1);
    // armTestTab.add("Ground Pickup", m_arm.requestMoveArmCommand(SuperStructurePosition.GroundPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.GroundPickup))).withPosition(2, 4).withSize(2, 1);
    // armTestTab.add("Substation Arm", m_arm.requestMoveArmCommand(SuperStructurePosition.SubstationPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.SubstationPickup))).withPosition(4, 4).withSize(2, 1);
    // armTestTab.add("Score Cube High", m_arm.requestMoveArmCommand(SuperStructurePosition.ScoreCubeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeHigh))).withPosition(6, 4).withSize(2, 1);
    // armTestTab.add("Score Cube Mid", m_arm.requestMoveArmCommand(SuperStructurePosition.ScoreCubeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeMid))).withPosition(8, 4).withSize(2, 1);
    // armTestTab.add("Score Cone High", m_arm.requestMoveArmCommand(SuperStructurePosition.ScoreConeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeHigh))).withPosition(0, 5).withSize(2, 1);
    // armTestTab.add("Score Cone Mid", m_arm.requestMoveArmCommand(SuperStructurePosition.ScoreConeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeMid))).withPosition(2, 5).withSize(2, 1);
    // armTestTab.add("Score Low", m_arm.requestMoveArmCommand(SuperStructurePosition.ScoreLow).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreLow))).withPosition(4, 5).withSize(2, 1);
    // armTestTab.add("Manual", m_arm.requestMoveArmCommand(SuperStructurePosition.Manual).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.Manual))).withPosition(6, 5).withSize(2, 1);
    armTestTab.add("Super Position", "None yet").withPosition(8, 5).withSize(2, 1);
    
    armTestTab.add("Arm Stow MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.Stowed).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.Stowed)))                   .withPosition(0, 3).withSize(2, 1);
    armTestTab.add("Arm Ground MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.GroundPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.GroundPickup)))     .withPosition(2, 3).withSize(2, 1);
    armTestTab.add("Arm Sub MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.SubstationPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.SubstationPickup))).withPosition(4, 3).withSize(2, 1);
    armTestTab.add("Arm Cube H MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreCubeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeHigh)))   .withPosition(6, 3).withSize(2, 1);
    armTestTab.add("Arm Cube M MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreCubeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeMid)))     .withPosition(0, 4).withSize(2, 1);
    armTestTab.add("Arm Cone H MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreConeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeHigh)))   .withPosition(2, 4).withSize(2, 1);
    armTestTab.add("Arm Cone M MM Test", new ArmDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreConeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeMid)))     .withPosition(4, 4).withSize(2, 1);
    
    armTestTab.add("Wrist Stow MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.Stowed).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.Stowed)))                   .withPosition(0, 5).withSize(2, 1);
    armTestTab.add("Wrist Ground MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.GroundPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.GroundPickup)))     .withPosition(2, 5).withSize(2, 1);
    armTestTab.add("Wrist Sub MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.SubstationPickup).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.SubstationPickup))).withPosition(4, 5).withSize(2, 1);
    armTestTab.add("Wrist Cube H MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreCubeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeHigh)))   .withPosition(6, 5).withSize(2, 1);
    armTestTab.add("Wrist Cube M MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreCubeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreCubeMid)))     .withPosition(0, 6).withSize(2, 1);
    armTestTab.add("Wrist Cone H MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreConeHigh).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeHigh)))   .withPosition(2, 6).withSize(2, 1);
    armTestTab.add("Wrist Cone M MM Test", new WristDriveToPositionPIDTest(m_arm, SuperStructurePosition.ScoreConeMid).unless(()->m_arm.isTransitionInvalid(SuperStructurePosition.ScoreConeMid)))     .withPosition(4, 6).withSize(2, 1);

    // armTestTab.add("Stow Arm Test", new SetArmRequestedPosition(m_arm, SuperStructurePosition.Stowed)).withPosition(0, 4).withSize(2, 1);
    // armTestTab.add("Ground_Pickup Arm Test", new SetArmRequestedPosition(m_arm, SuperStructurePosition.GroundPickup)).withPosition(2, 4).withSize(2, 1);
    // armTestTab.add("Substation Arm Test", new SetArmRequestedPosition(m_arm, SuperStructurePosition.SubstationPickup)).withPosition(4, 4).withSize(2, 1);
    // armTestTab.add("Cube Score High Arm Test", new SetArmRequestedPosition(m_arm, SuperStructurePosition.ScoreCubeHigh)).withPosition(6, 4).withSize(2, 1);
    // armTestTab.add("Set Current Pos to Req Pos", new SetCurrentPosToRequestedPosTest(m_arm)).withPosition(8, 4).withSize(2, 1);
    // armTestTab.add("Max Speed", 0).withPosition(0, 2).withSize(1,1);
    // armTestTab.add("Reset Max Speed", m_drivetrain.setMaxValue()).withPosition(1, 2).withSize(2, 1);
    // // Set the max speed variables
    // armTestTab.addDouble("Current Speed", m_drivetrain::get_max_speed).withPosition(0, 3).withSize(1,1);
    // armTestTab.add("Set_Max Speed", new SetMaxSpeedCommand(m_drivetrain)).withPosition(1, 3).withSize(2, 1);

    // armTestTab.add("Robot Heading", 0).withPosition(0, 0).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    
  }

  private void buildBangBangControlTab() {
    ShuffleboardTab bangbangtesttab = Shuffleboard.getTab("Bang Bang Control Test");

    bangbangtesttab.add("Setpoint", 0).withPosition(0, 0).withSize(1, 1);
    bangbangtesttab.add("Output %", 0).withPosition(1, 0).withSize(1, 1);
    bangbangtesttab.add("Test Pitch", 0).withPosition(0, 1).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider);
    bangbangtesttab.add(new BalanceBangBangCommandTest(m_drivetrain)).withPosition(0, 2).withSize(1, 1);
  }
}

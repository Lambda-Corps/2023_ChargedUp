// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.event.PrintEvent;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autoCommands.Pos1ScoreMove;
import frc.robot.autoCommands.Pos1ScoreMoveBalance;
import frc.robot.autoCommands.Pos2ScoreMoveBalance;
import frc.robot.autoCommands.Pos3ScoreMove;
import frc.robot.autoCommands.Pos3ScoreMoveBalance;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmDriveToPositionPIDTest;
import frc.robot.subsystems.Arm.ArmThenWristSequenceCommand;
import frc.robot.subsystems.Arm.DeployToGroundPickup;
import frc.robot.subsystems.Arm.DriveArmManually;
import frc.robot.subsystems.Arm.MoveArmToPositionMM;
import frc.robot.subsystems.Arm.MoveWristToPositionMM;
import frc.robot.subsystems.Arm.StowSuperStructure;
import frc.robot.subsystems.Arm.WristDriveToPositionPIDTest;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommand;
import frc.robot.subsystems.Arm.WristThenArmSequenceCommandTest;
import frc.robot.subsystems.Arm.Arm.ArmState;
import frc.robot.subsystems.Arm.Arm.SuperStructurePosition;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.DriveTrain.AlignToConeTapeWithVision;
import frc.robot.subsystems.DriveTrain.BalanceBangBangTestCommand;
import frc.robot.subsystems.DriveTrain.DefaultDriveTrainCommand;
import frc.robot.subsystems.DriveTrain.DriveMotionMagicTest;
import frc.robot.subsystems.DriveTrain.DriveTrain;
import frc.robot.subsystems.DriveTrain.FineGrainedDrivingControl;
import frc.robot.subsystems.DriveTrain.SetMaxSpeedCommand;
import frc.robot.subsystems.DriveTrain.SubStationDriveStop2feet;
import frc.robot.subsystems.DriveTrain.SubstationPickupDistanceRangefinder;
import frc.robot.subsystems.DriveTrain.TurnToAngleWithGyroPID;
import frc.robot.subsystems.DriveTrain.DriveDistanceInInchesTest;
import frc.robot.subsystems.DriveTrain.DriveMotionMagic;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.RunMotorsBackward;
import frc.robot.subsystems.Gripper.RunMotorsForward;
import frc.robot.subsystems.LEDs.LED;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  // private final Vision m_vision = new Vision();
  private final Arm m_arm = new  Arm();
  private final Gripper m_gripper = new  Gripper();
  private final LED m_led = new LED();
  



  // private final LED m_led = new LED();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private SendableChooser<Command> m_auto_chooser;
  public RobotContainer() {
    
    buildDriveTab();
    // buildDriveTestTab();
     buildArmTestTab();
    // Set subsystem default commands
    m_drivetrain.setDefaultCommand(new DefaultDriveTrainCommand(m_drivetrain, m_driver_controller));    
    // m_arm.setDefaultCommand(new DriveArmManually(m_arm, m_partner_controller));
    // Configure the button bindings
    configureButtonBindings();

    // Set the LED to rainbow as the default
    // m_led.setLED(LED.TOP_LEFT, LED.RAINBOW_FUNCTION);
    m_led.setLED(LED.ALL, LED.RAINBOW);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_partner_controller.rightBumper().whileTrue(m_arm.set_state(ArmState.Moving).andThen(
        new DriveArmManually(m_arm, m_partner_controller)).withName("Drive Manually")
    );
    // Start button
    // m_partner_controller.start().onTrue(
    //     m_arm.stopArmAndWristCommand()
    // );
    // // Left stick
    // m_partner_controller.leftStick().onTrue(
    //     Commands.run(() -> m_gripper.close_gripper()
    // );
    // m_partner_controller.leftStick().onTrue(m_gripper.contractGripperCommand().andThen(m_gripper.holdGamePieceCommand()));
    m_partner_controller.rightStick().onTrue(m_gripper.contractGripperCommand().andThen(m_gripper.holdGamePieceCommand()));
    // Right stick
    // m_partner_controller.rightStick().onTrue(
    //     Commands.run(() -> m_gripper.open_gripper())
    // );
    m_partner_controller.leftStick().onTrue(m_gripper.expandGripperCommand().andThen(m_gripper.holdGamePieceCommand()));
    // Right trigger
    m_partner_controller.rightTrigger().whileTrue(
        new RunMotorsForward(m_gripper)
    );
    // Left trigger
    m_partner_controller.leftTrigger().whileTrue(
        new RunMotorsBackward(m_gripper)
    );
    // X button
    m_partner_controller.x().onTrue(
      // new ConditionalCommand(
      //     new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeMid),
      //     new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreCubeMid),
      //     () -> m_arm.isBackwardMovement(SuperStructurePosition.ScoreCubeMid)
      // )
      new MoveWristToPositionMM(m_arm, SuperStructurePosition.ScoreCubeMid)
    );
    // Y button
    m_partner_controller.y().onTrue(
      new WristThenArmSequenceCommandTest(m_arm, SuperStructurePosition.ScoreCubeHigh)
    );
    // A button
    m_partner_controller.a().onTrue(
      // new ConditionalCommand(
      //     new ArmThenWristSequenceCommand(m_arm, SuperStructurePosition.ScoreConeMid),
      //     new WristThenArmSequenceCommand(m_arm, SuperStructurePosition.ScoreConeMid),
      //     () -> m_arm.isBackwardMovement(SuperStructurePosition.ScoreConeMid)
      // )
      new WristThenArmSequenceCommandTest(m_arm, SuperStructurePosition.ScoreConeMid)
  );
    // B button
    // m_partner_controller.b().onTrue(
    //     new PrintCommand("Cone High")
    // );

    // D-pad down
    m_partner_controller.povDown().onTrue(
      new StowSuperStructure(m_arm)
    );
    // D-pad up
    m_partner_controller.povUp().onTrue(
      new MoveWristToPositionMM(m_arm, SuperStructurePosition.SubstationPickup)
      // m_arm.moveWristToPositionMM(SuperStructurePosition.SubstationPickup, () -> !(m_arm.isBackwardMovement(SuperStructurePosition.SubstationPickup)))

    );
    // D-pad left
    m_partner_controller.povLeft().onTrue(
      new MoveWristToPositionMM(m_arm, SuperStructurePosition.ScoreLow)
      // m_arm.moveWristToPositionMM(SuperStructurePosition.ScoreLow, () -> !(m_arm.isBackwardMovement(SuperStructurePosition.ScoreLow)))

    );
    m_driver_controller.leftBumper().onTrue(m_drivetrain.shiftToHighGear());
    m_driver_controller.leftBumper().onFalse(m_drivetrain.shiftToLowGear());
    m_driver_controller.leftTrigger().whileTrue(new FineGrainedDrivingControl(m_drivetrain, m_driver_controller));
    m_driver_controller.rightTrigger().onTrue(m_gripper.contractGripperCommand().andThen(new WaitCommand(0.3)).andThen(new DeployToGroundPickup(m_arm, SuperStructurePosition.GroundPickup)).andThen(m_gripper.expandGripperCommand().andThen(m_gripper.holdGamePieceCommand())));
    m_driver_controller.rightTrigger().onFalse(m_gripper.contractGripperCommand().andThen(m_gripper.holdGamePieceCommand()).andThen(new StowSuperStructure(m_arm)));
   /*  m_driver_controller.a().whileTrue(new AlignToConeTapeWithVision(m_drivetrain, m_vision, m_driver_controller)); */
    m_driver_controller.x().onTrue(m_led.ResendLEDBytes());
    m_driver_controller.povUp().onTrue(new SubstationPickupDistanceRangefinder(m_drivetrain));
    m_driver_controller.povDown().onTrue(new SubStationDriveStop2feet(m_drivetrain, m_driver_controller));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand1() {
    // An ExampleCommand will run in autonomous
    return m_auto_chooser.getSelected();
  }
 
  private void buildDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab(("Drive Tab"));

    driveTab.add("ArmEncoder", 0).withPosition(2, 0).withSize(1, 1);
    driveTab.add("WristEncoder", 0).withPosition(2, 1).withSize(1, 1);
    driveTab.addBoolean("ArmForward", m_arm::getArmForwardLimit).withPosition(3, 0).withSize(1,1);
    driveTab.addBoolean("ArmReverse", m_arm::getArmReverseLimit).withPosition(4, 0).withSize(1,1);
    driveTab.addBoolean("WristForward", m_arm::getWristForwardLimit).withPosition(3, 1).withSize(1,1);
    driveTab.addBoolean("WristReverse", m_arm::getWristReverseLimit).withPosition(4, 1).withSize(1,1);
    // driveTab.addNumber("Curr Heading", m_drivetrain::getScaledHeading).withPosition(7,0).withSize(1,1);
   
    driveTab.add("Arm", m_arm).withPosition(0, 1).withSize(2, 1);
    driveTab.add("Gripper", m_gripper).withPosition(0, 2).withSize(2, 1);

    // driveTab.add("Camera", m_drivetrain);

    //delete this
    driveTab.add("Drive Till 2 feet from Wall", new SubStationDriveStop2feet(m_drivetrain, m_driver_controller)).withPosition(2, 2).withSize(2, 1);
    driveTab.add("Drive to substation pickup", new SubstationPickupDistanceRangefinder(m_drivetrain)).withPosition(4, 2).withSize(2, 1);
    driveTab.addDouble("UltraSonic sensor voltage", m_drivetrain::getRangeFinderValue).withPosition(2, 3).withSize(2, 1);;



    //Auto Options
    m_auto_chooser = new SendableChooser<Command>();
    driveTab.add("Autonomous Chooser", m_auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(0, 0).withSize(2, 1);
    m_auto_chooser.addOption("1 Score Balance", new Pos1ScoreMoveBalance(m_drivetrain, m_arm, m_gripper));
    m_auto_chooser.addOption("2 Score Balance", new Pos2ScoreMoveBalance(m_drivetrain, m_arm, m_gripper));
    m_auto_chooser.addOption("3 Score Balance", new Pos3ScoreMoveBalance(m_drivetrain, m_arm, m_gripper));
    m_auto_chooser.addOption("1 Score Move", new Pos1ScoreMove(m_drivetrain, m_gripper));
    m_auto_chooser.addOption("3 Score Move", new Pos3ScoreMove(m_drivetrain, m_gripper));
    m_auto_chooser.setDefaultOption("Default Auto incase we forget", new DriveMotionMagic(m_drivetrain, -150));
    m_auto_chooser.addOption("Drive Forward 14ft", new DriveMotionMagic(m_drivetrain, 150));
    m_auto_chooser.addOption("Drive Backward 14ft", new DriveMotionMagic(m_drivetrain, -150));
    
  }
  public Command getAutonomousCommand2() {
    // An ExampleCommand will run in autonomous
    return new Pos1ScoreMoveBalance(m_drivetrain, m_arm, m_gripper);
  }
  public Command getAutonomousCommand3() {
    // An ExampleCommand will run in autonomous
    return new Pos1ScoreMoveBalance(m_drivetrain, m_arm, m_gripper);
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
    driveTestTab.add("MM kD", 0).withPosition(5, 4).withSize(1,1);
    driveTestTab.add("Drive MM", new DriveMotionMagicTest(m_drivetrain)).withPosition(6,3).withSize(2, 1);
    driveTestTab.add("Drive PID", new DriveDistanceInInchesTest(m_drivetrain)).withPosition(6,3).withSize(2, 1);
    driveTestTab.addDouble("Left Error", m_drivetrain::getLeftError).withPosition(0,4).withSize(1,1);
    driveTestTab.addDouble("Right Error", m_drivetrain::getRightError).withPosition(1,4).withSize(1,1);
    driveTestTab.add("Target Degrees", 0).withPosition(6, 1).withSize(1, 1);
    driveTestTab.addNumber("Curr Heading", m_drivetrain::getScaledHeading).withPosition(4,0).withSize(1,1);
    driveTestTab.add("Gyro", m_drivetrain.getGyro()).withPosition(3, 1).withSize(2, 2).withWidget(BuiltInWidgets.kGyro);
    driveTestTab.addNumber("Yaw", m_drivetrain::getYaw).withPosition(0, 2).withSize(1, 1);
    driveTestTab.addNumber("Pitch", m_drivetrain::getPitch).withPosition(1, 2).withSize(1, 1);
    driveTestTab.addNumber("Roll", m_drivetrain::getRoll).withPosition(2, 2).withSize(1, 1);
    driveTestTab.addNumber("Angle", m_drivetrain::getAngle).withPosition(5, 2).withSize(1, 1);
    driveTestTab.add("Target in Ticks", 0).withPosition(6, 2).withSize(1, 1);
    driveTestTab.addNumber("DriveTrain Setpoint", m_drivetrain::get_setpoint).withPosition(7, 2).withSize(1, 1);
    driveTestTab.add("Reset Drivetrain setpoint", m_drivetrain.reset_dt_setpoint()).withPosition(8, 2).withSize(2, 1);

    driveTestTab.add("Bang Bang Forward Speed", 0).withPosition(0, 5).withSize(1, 1);
    driveTestTab.add("Bang Bang Reverse Speed", 0).withPosition(1, 5).withSize(1, 1);
    driveTestTab.add("Bang Bang Target Pitch", 0).withPosition(2, 5).withSize(1, 1);
    driveTestTab.add("Fwd Out", 0).withPosition(3, 4).withSize(1, 1);
    driveTestTab.add("Rev Out", 0).withPosition(3, 5).withSize(1, 1);
    driveTestTab.add("Bang Bang Test Pitch Value", 0).withPosition(4, 5).withSize(2, 1).withWidget(BuiltInWidgets.kNumberSlider);

    // PID Tuning
    driveTestTab.add("Turn PID", m_drivetrain.get_dt_turn_pidcontroller()).withPosition(5, 0);
    driveTestTab.add("Turn with PID", new DriveDistanceInInchesTest(m_drivetrain)).withPosition(6, 0).withSize(2, 1);
    driveTestTab.add("Drive Fine Grained", new FineGrainedDrivingControl(m_drivetrain, m_driver_controller)).withPosition(8, 1).withSize(2, 1);
    driveTestTab.add("BangBang Command", new BalanceBangBangTestCommand(m_drivetrain).andThen(new TurnToAngleWithGyroPID(m_drivetrain, 90))).withPosition(6, 5 ).withSize(2, 1); 

    driveTestTab.add("DriveSlowly Command", m_drivetrain.driveSlowlyUntil().withTimeout(2).andThen(m_drivetrain.stopMotorsCommand()));
  }

  private void buildArmTestTab() {
    ShuffleboardTab armTestTab = Shuffleboard.getTab("Arm Test");

    armTestTab.add("ArmEncoder", 0).withPosition(0, 0).withSize(1, 1);
    armTestTab.addBoolean("ArmForward", m_arm::getArmForwardLimit).withPosition(1, 0).withSize(1,1);
    armTestTab.addBoolean("ArmReverse", m_arm::getArmReverseLimit).withPosition(2, 0).withSize(1,1);
  
    armTestTab.add("WristEncoder", 0).withPosition(0, 1).withSize(1, 1);

    armTestTab.addBoolean("WristForward", m_arm::getWristForwardLimit).withPosition(1, 1).withSize(1,1);
    armTestTab.addBoolean("WristReverse", m_arm::getWristReverseLimit).withPosition(2, 1).withSize(1,1);
    armTestTab.add("Arm", m_arm).withPosition(8, 0).withSize(2, 1);
    armTestTab.add("Gripper", m_gripper).withPosition(8, 1).withSize(2, 1);
    armTestTab.add("Super Position", "None yet").withPosition(8, 2).withSize(2, 1);

    // PIDF Values
    armTestTab.add("Arm kP", 0).withPosition(4, 0).withSize(1, 1);
    armTestTab.add("Arm kF", 0).withPosition(5, 0).withSize(1, 1);
    armTestTab.add("Arm Velo", 0).withPosition(6, 0).withSize(1, 1);
    armTestTab.add("Arm Error", 0).withPosition(7, 0).withSize(1, 1);
    armTestTab.add("Wrist kP", 0).withPosition(4, 1).withSize(1, 1);
    armTestTab.add("Wrist kF", 0).withPosition(5, 1).withSize(1, 1);
    armTestTab.add("Wrist Velo", 0).withPosition(6, 1).withSize(1, 1);
    armTestTab.add("Wrist Error", 0).withPosition(7, 1).withSize(1, 1);

    // Add the commands to the page
    armTestTab.add("Zero Wrist Encoder", m_arm.setWristEncoderToZero()).withPosition(0, 2).withSize(2, 1);
    armTestTab.add("Zero Arm Encoder", m_arm.setArmEncoderToZero()).withPosition(2, 2).withSize(2, 1);
   
    armTestTab.add("Cone Mid Test", new WristThenArmSequenceCommandTest(m_arm, SuperStructurePosition.ScoreConeMid)).withPosition(4, 3).withSize(2, 1);
    armTestTab.add("Stow Test", new WristThenArmSequenceCommandTest(m_arm, SuperStructurePosition.Stowed)).withPosition(6, 3).withSize(2, 1);
    armTestTab.add("Cube High Test", new WristThenArmSequenceCommandTest(m_arm, SuperStructurePosition.ScoreCubeHigh)).withPosition(8, 3).withSize(2, 1);
  }
}

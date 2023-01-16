// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import frc.robot.subsystems.DriveTrain.DriveTrain;

/** Add your docs here. */
class DriveTrainTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    DriveTrain dt;

    TalonFXSimCollection m_left_motors, m_right_motors;
    DifferentialDrivetrainSim dt_sim;
    @BeforeEach // this method will run before each test
    void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        dt = new DriveTrain();

        m_left_motors = dt.getLeftSimCollection();
        m_right_motors = dt.getRightSimCollection();
    }

    @AfterEach // Cleanup any things we need
    void shutdown(){
        // Do stuff if we need
    }

    @Test
    void emptyExampleTestCase(){
        // Test cases must "enable" the robot in simulation for the motors to actually "drive"
        waitForUpdate(); // This enables the CTRE simulation motors

        // Call the drivetrain function you want to test (e.g. teleop_drive)
        dt.teleop_drive(0, 0);

        // The drivetrain still needs to accelerate to get to speed, so you can step through
        // the simulation that advance 20ms.  The waitForUpdate() command will sleep the thread
        // of execution for 200 ms.  So, if we loop the waitForUpdate() method 5 times, it will
        // simulate 1 full second in time.  To equally simulate the drivetrain action, the 
        // simulation periodic method must be called 10 times to step.  The following loop will
        // simulate 1 second of driving.
        for (var i = 0; i < 5; i++){
            for( var j = 0; j < 10; j++){
                // Step the simulator 200 ms
                dt.simulationPeriodic();    
            }
             /* Wait 200 ms for ramping to take affect */
            waitForUpdate();
        }

        // We drove with 0 and 0 as inputs, the robot should not be moving
        assertEquals(0.0, dt.getLeftSpeed(), DELTA);
        assertEquals(0.0, dt.getRightSpeed(), DELTA);
    }

    @Test
    void driveTrainConstructorWorks(){
        assertNotEquals(dt, null);
    }

    @Test
    void teleopDriveFullBackward(){
        double throt = -1.0;
        double turn = 0.0;
        waitForUpdate();
        dt.teleop_drive(throt, turn);   

        for (var i = 0; i < 5; i++){
            for( var j = 0; j < 10; j++){
                // Step the simulator 200 ms
                dt.simulationPeriodic();    
            }
             /* Wait 200 ms for ramping to take affect */
            waitForUpdate();
        }
       
        // double expectedLeft = busV * (throt - turn);
        // double expectedRight = busV * (throt + turn);
        // if (Constants.Left_Side_Inverted) expectedLeft = -expectedLeft;
        // if (Constants.Right_Side_Inverted) expectedRight = -expectedRight;
        assertTrue(dt.getLeftSpeed() < 0);
        assertTrue(dt.getRightSpeed() < 0);
        assertTrue(m_left_motors.getMotorOutputLeadVoltage() < 0);
        assertTrue(m_right_motors.getMotorOutputLeadVoltage() > 0);
    }

    @Test
    void teleopDriveFullForward(){
        waitForUpdate();
        double throt = 1.0;
        double turn = 0.0;
        dt.teleop_drive(throt, turn);

        for (var i = 0; i < 5; i++){
            for( var j = 0; j < 10; j++){
                // Step the simulator 200 ms
                dt.simulationPeriodic();    
            }
             /* Wait 200 ms for ramping to take affect */
            waitForUpdate();
        }

        // Need to find a way to verify that each motor controller is sending
        // the motors in the proper direction
        assertTrue(dt.getLeftSpeed() > 0);
        assertTrue(dt.getRightSpeed() > 0);
        assertTrue(m_left_motors.getMotorOutputLeadVoltage() > 0);
        assertTrue(m_right_motors.getMotorOutputLeadVoltage() < 0);
    }

    private static void waitForUpdate() {
        try {
            com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(500);
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}

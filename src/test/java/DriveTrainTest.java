// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.*;
import edu.wpi.first.hal.HAL;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import frc.robot.subsystems.DriveTrain.DriveTrain;

/** Add your docs here. */
class DriveTrainTest {
    static final double DELTA = 1e-2; // acceptable deviation range
    DriveTrain dt;

    TalonFXSimCollection m_left_motors, m_right_motors;

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
    void driveTrainConstructorWorks(){
        assertNotEquals(dt, null);
    }

    @Test
    void teleopDriveFullForward(){
        dt.teleop_drive(1, 0);
        assertTrue(dt.getLeftSpeed() > 0);
        assertTrue(dt.getRightSpeed() > 0);
    }
}

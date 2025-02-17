// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.RotateArmSubsystem;

/** Add your docs here. */
public class ShuffleboardContent {

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initLowerArm(RotateArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Arms");

                drLayout1.addNumber("Lower Arm Angle", () -> drive.getPosition())
                        .withPosition(9, 0)
                        .withSize(8, 1);
                drLayout1.addNumber("Lower Current Rotations", () -> drive.rotations)
                        .withPosition(9, 3)
                        .withSize(8, 1);
                drLayout1.addNumber("Lower Min Rotations", () -> drive.getMinRotations())
                        .withPosition(9, 6)
                        .withSize(8, 1);
                drLayout1.addNumber("Lower Max Rotations", () -> drive.getMaxRotations())
                        .withPosition(9, 9)
                        .withSize(8, 1);
        }

}

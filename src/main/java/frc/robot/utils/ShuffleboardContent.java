// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.BallScrewSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LinearArmSubsystem;
import frc.robot.subsystems.RotateArmSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.RotateAlgaeSubsystem;

/** Add your docs here. */
public class ShuffleboardContent {

        static ShuffleboardLayout boolsLayout;

        public ShuffleboardContent() {

        }

        public static void initRotateArm(RotateArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Rotate Arm");

                drLayout1.addNumber("Angle", () -> Units.radiansToDegrees(drive.getPosition()))
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> Units.radiansToDegrees(drive.m_positionRadians))
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> Units.radiansToDegrees(drive.getMinRotations()))
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> Units.radiansToDegrees(drive.getMaxRotations()))
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initBallScrew(BallScrewSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Ball Screw");

                drLayout1.addNumber("Distance", () -> drive.getPosition())
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Distance", () -> drive.m_positionInches)
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Distance", () -> drive.getMinDistance())
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Distance", () -> drive.getMaxDistance())
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initLinearArm(LinearArmSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Linear Arm");

                drLayout1.addNumber("Angle", () -> Units.radiansToDegrees(drive.getPosition()))
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> Units.radiansToDegrees(drive.m_positionRadians))
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> Units.radiansToDegrees(drive.getMinRotations()))
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> Units.radiansToDegrees(drive.getMaxRotations()))
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initClimber(ClimberSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Climber");

                drLayout1.addNumber("Angle", () -> Units.radiansToDegrees(drive.getPosition()))
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> Units.radiansToDegrees(drive.m_positionRadians))
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> Units.radiansToDegrees(drive.getMinRotations()))
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> Units.radiansToDegrees(drive.getMaxRotations()))
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initRotateAlgae(RotateAlgaeSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Algae");

                drLayout1.addNumber("Angle", () -> Units.radiansToDegrees(drive.getPosition()))
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> Units.radiansToDegrees(drive.m_positionRadians))
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> Units.radiansToDegrees(drive.getMinRotations()))
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> Units.radiansToDegrees(drive.getMaxRotations()))
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

        public static void initWrist(WristSubsystem drive) {
                ShuffleboardTab drLayout1 = Shuffleboard.getTab("Wrist");

                drLayout1.addNumber("Angle", () -> Units.radiansToDegrees(drive.getPosition()))
                        .withPosition(1, 1)
                        .withSize(2, 1);
                drLayout1.addNumber("Target Angle", () -> Units.radiansToDegrees(drive.m_positionRadians))
                        .withPosition(1, 2)
                        .withSize(2, 1);
                drLayout1.addNumber("Min Angle", () -> Units.radiansToDegrees(drive.getMinRotations()))
                        .withPosition(1, 3)
                        .withSize(2, 1);
                drLayout1.addNumber("Max Angle", () -> Units.radiansToDegrees(drive.getMaxRotations()))
                        .withPosition(1, 4)
                        .withSize(2, 1);
        }

}

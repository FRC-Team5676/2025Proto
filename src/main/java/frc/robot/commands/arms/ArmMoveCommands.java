package frc.robot.commands.arms;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallScrewSubsystem;

public class ArmMoveCommands extends Command {

    private final BallScrewSubsystem m_BallScrew;
    private final ArmSubsystem m_Arm;

    public ArmMoveCommands(BallScrewSubsystem ballScrew,
            ArmSubsystem arm) {

        m_BallScrew = ballScrew;
        m_Arm = arm;

        addRequirements(ballScrew, arm);
    }

    public Command MoveAllHome() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveWrist(0)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMaxLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveRotateArm(0))
            ),
            Commands.waitSeconds(1),
            Commands.run(m_BallScrew::moveToDownPosition, m_BallScrew)
        );
    }

    public Command MoveToL2() {
        double timeOut = 0.0;
        if (m_BallScrew.getPosition() > -250000) {
            timeOut = 1.0;
        }
        return Commands.sequence(
            Commands.run(m_BallScrew::moveToUpPosition, m_BallScrew),
            Commands.waitSeconds(timeOut),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveWrist(0)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMaxLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveRotateArm(72))
            )
        );
    }

    public Command MoveToL3() {
        double timeOut = 0.0;
        if (m_BallScrew.getPosition() > -250000) {
            timeOut = 1.0;
        }
        return Commands.sequence(
            Commands.run(m_BallScrew::moveToUpPosition, m_BallScrew),
            Commands.waitSeconds(timeOut),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveWrist(0)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMinLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveRotateArm(-72))
            )
        );
    }

    public Command MoveToL4() {
        double timeOut = 0.0;
        if (m_BallScrew.getPosition() > -250000) {
            timeOut = 1.0;
        }
        return Commands.sequence(
            Commands.run(m_BallScrew::moveToUpPosition, m_BallScrew),
            Commands.waitSeconds(timeOut),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveWrist(0)),
                new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getMaxLinearArmDegrees())),
                new InstantCommand(() -> m_Arm.moveRotateArm(-72))
            )
        );
    }

}

package frc.robot.commands.arms;

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

    public Command moveToHome() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-28)),
                new InstantCommand(() -> m_Arm.moveWrist(90))
            ),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_Arm.moveLinearArm(m_Arm.getPickupLinearArmDegrees())),
            Commands.waitSeconds(0.5),
            new InstantCommand(() -> m_BallScrew.moveToDownPosition())
        );
    }

    public Command pickupCoral() {
        return Commands.sequence(
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(0)),
                new InstantCommand(() -> m_Arm.moveWrist(120))
            ),
            Commands.waitSeconds(0.75),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-28)),
                new InstantCommand(() -> m_Arm.moveWrist(90))
            )
        );
}

    public Command moveToL2() {
        return moveToHome();
    }

    public Command moveToL3() {
        return moveToHome();
    }

    public Command moveToL4() {
        return Commands.sequence(
            new InstantCommand(() -> m_BallScrew.moveToUpPosition()),
            Commands.waitSeconds(1),
            Commands.parallel(
                new InstantCommand(() -> m_Arm.moveRotateArm(-160)),
                new InstantCommand(() -> m_Arm.moveLinearArm(-782)),
                new InstantCommand(() -> m_Arm.moveWrist(-135))
            )
        );
    }

}

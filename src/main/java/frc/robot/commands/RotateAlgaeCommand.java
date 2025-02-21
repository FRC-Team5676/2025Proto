// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.RotateAlgae;

public class RotateAlgaeCommand extends Command {

    private final RotateAlgae m_controlArm;
    private final CommandXboxController m_controller;

    /** Driver control */
    public RotateAlgaeCommand(RotateAlgae controlArm, CommandXboxController controller) {
        m_controlArm = controlArm;
        m_controller = controller;

        addRequirements(controlArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_controlArm.driveArm(m_controller.getRightX());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
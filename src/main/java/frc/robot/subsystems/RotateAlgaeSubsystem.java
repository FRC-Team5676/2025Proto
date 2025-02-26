// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class RotateAlgaeSubsystem extends SubsystemBase {

  private double m_targetRadians;

  private static final double kGearRatio = 45/1;
  private static final double kIntakeArmEncoderPositionFactor = (2 * Math.PI) / kGearRatio;

  private final int m_RotateAlgaeCanId = 55;
  private final int m_intakeAlgaeCanId = 51;

  private final RelativeEncoder m_rotateEncoder;
  private final SparkMax m_rotateMotor;
  private final SparkClosedLoopController m_rotateController;

  private final double minRadians = Units.degreesToRadians(-300);
  private final double maxRadians = Units.degreesToRadians(0);

  private final TalonFX m_intakeMotor;

  public RotateAlgaeSubsystem() {
    // Rotate Motor setup
    m_rotateMotor = new SparkMax(m_RotateAlgaeCanId, MotorType.kBrushless);

    // Rotate encoder setup
    m_rotateEncoder = m_rotateMotor.getEncoder();

    m_rotateController = m_rotateMotor.getClosedLoopController();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop
    .p(0.5)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    config.encoder.positionConversionFactor(kIntakeArmEncoderPositionFactor);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);
    m_rotateMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    m_targetRadians = m_rotateEncoder.getPosition();

    ShuffleboardContent.initRotateAlgae(this);

    m_intakeMotor = new TalonFX(m_intakeAlgaeCanId);
  }

  @Override
  public void periodic() {
  }

  public void intakeAlgea() {
    m_targetRadians = minRadians;
    setReferencePeriodic();
    m_intakeMotor.set(0.12);
  }

  public void stop() {
    m_targetRadians = maxRadians;
    setReferencePeriodic();
    m_intakeMotor.set(0);
  }

  public void shootAlgea() {
    m_targetRadians = maxRadians;
    setReferencePeriodic();
    m_intakeMotor.set(-0.1);
  }

  public double getMinDegrees() {
    return Units.radiansToDegrees(minRadians);
  }

  public double getMaxDegrees() {
    return Units.radiansToDegrees(maxRadians);
  }

  public double getActualDegrees() {
    return Units.radiansToDegrees(m_rotateEncoder.getPosition());
  }

  public double getTargetDegrees() {
    return Units.radiansToDegrees(m_targetRadians);
  }

  private void setReferencePeriodic() {
    m_targetRadians = MathUtil.clamp(m_targetRadians, minRadians, maxRadians);
    m_rotateController.setReference(m_targetRadians, ControlType.kPosition);
  }
}
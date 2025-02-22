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
import com.revrobotics.spark.config.SparkMaxConfig;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class RotateAlgaeSubsystem extends SubsystemBase {
  public double m_positionRadians;
  public static final double kGearRatio = 45/1;
  public static final double kIntakeArmEncoderPositionFactor = (2 * Math.PI) / kGearRatio;

  private final int m_RotateAlgaeCanId = 55;
  private final int m_intakeAlgaeCanId = 51;

  private final RelativeEncoder m_rotateEncoder;
  private final SparkMax m_rotateMotor;
  private final SparkClosedLoopController m_rotateController;

  private final double minRotations = Units.degreesToRadians(-300);
  private final double maxRotations = Units.degreesToRadians(0);

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

    m_rotateMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_positionRadians = m_rotateEncoder.getPosition();

    ShuffleboardContent.initRotateAlgae(this);

    m_intakeMotor = new TalonFX(m_intakeAlgaeCanId);
  }

  @Override
  public void periodic() {
  }

  public void intakeAlgea() {
    m_positionRadians = minRotations;
    setReferencePeriodic();
    m_intakeMotor.set(0.12);
  }

  public void stop() {
    m_positionRadians = maxRotations;
    setReferencePeriodic();
    m_intakeMotor.set(0);
  }

  public void shootAlgea() {
    m_positionRadians = maxRotations;
    setReferencePeriodic();
    m_intakeMotor.set(-0.1);
  }

  public double getMinRotations() {
    return minRotations;
  }

  public double getMaxRotations() {
    return maxRotations;
  }

  public double getPosition() {
    return m_rotateEncoder.getPosition();
  }

  public void setReferenceValue(double rotation) {
    m_positionRadians = Units.degreesToRadians(rotation);
  }

  public void setReferencePeriodic() {
    m_positionRadians = MathUtil.clamp(m_positionRadians, minRotations, maxRotations);
    m_rotateController.setReference(m_positionRadians, ControlType.kPosition);
  }
}


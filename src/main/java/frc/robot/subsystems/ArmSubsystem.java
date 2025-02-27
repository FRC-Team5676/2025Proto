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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class ArmSubsystem extends SubsystemBase {

  private double m_RotateArmTargetRadians;
  private double m_LinearArmTargetRadians;
  private double m_WristTargetRadians;

  private static final double kRotateArmGearRatio = 100 / 1 * 46 / 14;
  private static final double kRotateArmPositionFactor = (2 * Math.PI) / kRotateArmGearRatio;
  private static final double kLinearArmGearRatio = 20 / 1;
  private static final double kLinearArmPositionFactor = (2 * Math.PI) / kLinearArmGearRatio;
  private static final double kWristGearRatio = 2/1;
  private static final double kWristPositionFactor = (2 * Math.PI) / kWristGearRatio;

  private final int m_RotateArmCanId = 52;
  private final int m_LinearArmCanId = 53;
  private final int m_WristCanId = 54;

  private final RelativeEncoder m_RotateArmEncoder;
  private final RelativeEncoder m_LinearArmEncoder;
  private final RelativeEncoder m_WristEncoder;

  private final SparkMax m_RotateArmMotor;
  private final SparkMax m_LinearArmMotor;
  private final SparkMax m_WristMotor;

  private final SparkClosedLoopController m_RotateArmController;
  private final SparkClosedLoopController m_LinearArmController;
  private final SparkClosedLoopController m_WristController;

  private final double m_MinRotateArmRadians = Units.degreesToRadians(-270);
  private final double m_MaxRotateArmRadians = Units.degreesToRadians(270);
  private final double m_MinRotateZoneRadians = Units.degreesToRadians(-190);
  private final double m_MaxRotateZoneRadians = Units.degreesToRadians(80);

  private double m_ExtendedLinearArmRadians = Units.degreesToRadians(-720);
  private double m_PickupLinearArmRadians = Units.degreesToRadians(-434);
  private double m_L4LinearArmRadians = Units.degreesToRadians(-622);
  private double m_RetractedLinearArmRadians = Units.degreesToRadians(0);

  private final double m_MinWristRadians = Units.degreesToRadians(-180);
  private final double m_MaxWristRadians = Units.degreesToRadians(180);

  public ArmSubsystem() {
    // Motor setup
    m_RotateArmMotor = new SparkMax(m_RotateArmCanId, MotorType.kBrushless);
    m_LinearArmMotor = new SparkMax(m_LinearArmCanId, MotorType.kBrushless);
    m_WristMotor = new SparkMax(m_WristCanId, MotorType.kBrushed);

    // Encoder setup
    m_RotateArmEncoder = m_RotateArmMotor.getEncoder();
    m_LinearArmEncoder = m_LinearArmMotor.getEncoder();
    m_WristEncoder = m_WristMotor.getEncoder();

    // Controller setup
    m_RotateArmController = m_RotateArmMotor.getClosedLoopController();
    m_LinearArmController = m_LinearArmMotor.getClosedLoopController();
    m_WristController = m_WristMotor.getClosedLoopController();

    // Configure Controllers
    configControllers();

    // Set Initial Positions
    m_RotateArmTargetRadians = m_RotateArmEncoder.getPosition();
    m_LinearArmTargetRadians = m_LinearArmEncoder.getPosition();
    m_WristTargetRadians = m_WristEncoder.getPosition();

    // Setup Shuffleboard
    ShuffleboardContent.initRotateArm(this);
    ShuffleboardContent.initLinearArm(this);
    ShuffleboardContent.initWrist(this);
  }

  @Override
  public void periodic() {
  }

  // Rotate Arm
  public double getMinRotateArmDegrees() {
    return Units.radiansToDegrees(m_MinRotateArmRadians);
  }

  public double getMaxRotateArmDegrees() {
    return Units.radiansToDegrees(m_MaxRotateArmRadians);
  }

  public double getRotateArmDegrees() {
    return Units.radiansToDegrees(m_RotateArmEncoder.getPosition());
  }

  public double getRotateArmTargetDegrees() {
    return Units.radiansToDegrees(m_RotateArmTargetRadians);
  }

  // Linear Arm
  public double getExtendedLinearArmDegrees() {
    return Units.radiansToDegrees(m_ExtendedLinearArmRadians);
  }

  public double getPickupLinearArmDegrees() {
    return Units.radiansToDegrees(m_PickupLinearArmRadians);
  }

  public double getL4LinearArmDegrees() {
    return Units.radiansToDegrees(m_L4LinearArmRadians);
  }

  public double getRetractedLinearArmDegrees() {
    return Units.radiansToDegrees(m_RetractedLinearArmRadians);
  }

  public double getLinearArmDegrees() {
    return Units.radiansToDegrees(m_LinearArmEncoder.getPosition());
  }

  public double getLinearArmTargetDegrees() {
    return Units.radiansToDegrees(m_LinearArmTargetRadians);
  }

  // Wrist
  public double getMinWristDegrees() {
    return Units.radiansToDegrees(m_MinWristRadians);
  }

  public double getMaxWristDegrees() {
    return Units.radiansToDegrees(m_MaxWristRadians);
  }

  public double getWristDegrees() {
    return Units.radiansToDegrees(m_WristEncoder.getPosition());
  }

  public double getWristTargetDegrees() {
    return Units.radiansToDegrees(m_WristTargetRadians);
  }

  // Drive Rotate Arm
  public void driveRotateArm(double degrees) {
    if (Math.abs(degrees) > 0.05) {
      m_RotateArmTargetRadians += Units.degreesToRadians(degrees);

      m_RetractedLinearArmRadians += Units.degreesToRadians(degrees);
      m_PickupLinearArmRadians += Units.degreesToRadians(degrees);
      m_L4LinearArmRadians += Units.degreesToRadians(degrees);
      m_ExtendedLinearArmRadians += Units.degreesToRadians(degrees);
      m_LinearArmTargetRadians += Units.degreesToRadians(degrees);

      if (m_RotateArmTargetRadians >= m_MaxRotateZoneRadians || m_RotateArmTargetRadians <= m_MinRotateZoneRadians) {
        m_LinearArmTargetRadians = m_RetractedLinearArmRadians;
      } 
    }
    setReferencePeriodic();
  }

  public void moveRotateArm(double degrees) {
    double oldRadians = m_RotateArmTargetRadians;
    double newRadians = Units.degreesToRadians(degrees);
    double adjRadians = newRadians - oldRadians;

    m_RotateArmTargetRadians = newRadians;

    m_RetractedLinearArmRadians += adjRadians;
    m_PickupLinearArmRadians += adjRadians;
    m_L4LinearArmRadians += adjRadians;
    m_ExtendedLinearArmRadians += adjRadians;
    m_LinearArmTargetRadians += adjRadians;

    if (m_RotateArmTargetRadians >= m_MaxRotateZoneRadians || m_RotateArmTargetRadians <= m_MinRotateZoneRadians) {
      m_LinearArmTargetRadians = m_RetractedLinearArmRadians;
    } 

    setReferencePeriodic();
  }

  // Drive Linear Arm
  public void driveLinearArm(double degrees) {
    if (Math.abs(degrees) > 0.05) {
      m_LinearArmTargetRadians += Units.degreesToRadians(degrees * 10);
    }
    setReferencePeriodic();
  }

  public void moveLinearArmRetracted() {
    m_LinearArmTargetRadians = m_RetractedLinearArmRadians;
    setReferencePeriodic();
  }

  public void moveLinearArmPickup() {
    m_LinearArmTargetRadians = m_PickupLinearArmRadians;
    setReferencePeriodic();
  }

  public void moveLinearArmL4() {
    m_LinearArmTargetRadians = m_L4LinearArmRadians;
    setReferencePeriodic();
  }

  // Drive Wrist
  public void driveWrist(double degrees) {
    if (Math.abs(degrees) > 0.05) {
      m_WristTargetRadians += Units.degreesToRadians(degrees * 2);
    }
    setReferencePeriodic();
  }

  public void moveWrist(double degrees) {
    m_WristTargetRadians = Units.degreesToRadians(degrees);
    setReferencePeriodic();
  }

  // Private methods
  private void setReferencePeriodic() {
    m_RotateArmTargetRadians = MathUtil.clamp(m_RotateArmTargetRadians, m_MinRotateArmRadians, m_MaxRotateArmRadians);
    m_LinearArmTargetRadians = MathUtil.clamp(m_LinearArmTargetRadians, m_ExtendedLinearArmRadians, m_RetractedLinearArmRadians);
    m_WristTargetRadians = MathUtil.clamp(m_WristTargetRadians, m_MinWristRadians, m_MaxWristRadians);

    m_RotateArmController.setReference(m_RotateArmTargetRadians, ControlType.kPosition);
    m_LinearArmController.setReference(m_LinearArmTargetRadians, ControlType.kPosition);
    m_WristController.setReference(m_WristTargetRadians, ControlType.kPosition);
  }

  private void configControllers() {
    // Config Rotate Arm
    SparkMaxConfig configRotateArm = new SparkMaxConfig();
    configRotateArm.closedLoop
    .p(3)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    configRotateArm.encoder.positionConversionFactor(kRotateArmPositionFactor);
    configRotateArm.idleMode(IdleMode.kBrake);
    configRotateArm.smartCurrentLimit(40);
    m_RotateArmMotor.configure(configRotateArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Config Linear Arm
    SparkMaxConfig configLinearArm = new SparkMaxConfig();
    configLinearArm.closedLoop
    .p(1)
    .i(0)
    .d(0)
    .outputRange(-0.3, 0.3);
    configLinearArm.encoder.positionConversionFactor(kLinearArmPositionFactor);
    configLinearArm.idleMode(IdleMode.kBrake);
    configLinearArm.smartCurrentLimit(40);
    m_LinearArmMotor.configure(configLinearArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Config Wrist
    SparkMaxConfig configWrist = new SparkMaxConfig();
    configWrist.closedLoop
    .p(2)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    configWrist.encoder.positionConversionFactor(kWristPositionFactor);
    configWrist.idleMode(IdleMode.kBrake);
    configWrist.smartCurrentLimit(40);
    m_WristMotor.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShuffleboardContent;

public class ArmSubsystem extends SubsystemBase {

  public double m_RotateArmPositionRadians;
  public double m_LinearArmPositionRadians;
  public double m_WristPositionRadians;
  public static final double kRotateArmGearRatio = 100 / 1 * 46 / 14;
  public static final double kRotateArmPositionFactor = (2 * Math.PI) / kRotateArmGearRatio;
  public static final double kLinearArmGearRatio = 20 / 1;
  public static final double kLinearArmPositionFactor = (2 * Math.PI) / kLinearArmGearRatio;
  public static final double kWristGearRatio = 2/1;
  public static final double kWristPositionFactor = (2 * Math.PI) / kWristGearRatio;

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
  private final double m_MinLinearArmRadians = Units.degreesToRadians(-540);
  private final double m_MaxLinearArmRadians = Units.degreesToRadians(540);
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

    // Config Rotate Arm
    SparkMaxConfig configRotateArm = new SparkMaxConfig();
    configRotateArm.closedLoop
    .p(1.5)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    configRotateArm.encoder.positionConversionFactor(kRotateArmPositionFactor);
    m_RotateArmMotor.configure(configRotateArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Config Linear Arm
    SparkMaxConfig configLinearArm = new SparkMaxConfig();
    configLinearArm.closedLoop
    .p(0.5)
    .i(0)
    .d(0)
    .outputRange(-0.3, 0.3);
    configLinearArm.encoder.positionConversionFactor(kLinearArmPositionFactor);
    m_LinearArmMotor.configure(configLinearArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Config Wrist
    SparkMaxConfig configWrist = new SparkMaxConfig();
    configWrist.closedLoop
    .p(2)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    configWrist.encoder.positionConversionFactor(kWristPositionFactor);
    m_WristMotor.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Set Initial Positions
    m_RotateArmPositionRadians = m_RotateArmEncoder.getPosition();
    m_LinearArmPositionRadians = m_LinearArmEncoder.getPosition();
    m_WristPositionRadians = m_WristEncoder.getPosition();

    // Setup Shuffleboard
    ShuffleboardContent.initRotateArm(this);
    ShuffleboardContent.initLinearArm(this);
    ShuffleboardContent.initWrist(this);
  }

  @Override
  public void periodic() {
  }

  public void moveRotateArm(double degrees) {
    m_RotateArmPositionRadians = Units.degreesToRadians(degrees);
    setReferencePeriodic();
  }

  public void moveLinearArm(double degrees) {
    m_LinearArmPositionRadians = Units.degreesToRadians(degrees);
    setReferencePeriodic();
  }

  public void moveWrist(double degrees) {
    m_WristPositionRadians = Units.degreesToRadians(degrees);
    setReferencePeriodic();
  }

  public double getMinRotateArmDegrees() {
    return Units.radiansToDegrees(m_MinRotateArmRadians);
  }

  public double getMinLinearArmDegrees() {
    return Units.radiansToDegrees(m_MinLinearArmRadians);
  }

  public double getMinWristDegrees() {
    return Units.radiansToDegrees(m_MinWristRadians);
  }

  public double getMaxRotateArmDegrees() {
    return Units.radiansToDegrees(m_MaxRotateArmRadians);
  }

  public double getMaxLinearArmDegrees() {
    return Units.radiansToDegrees(m_MaxLinearArmRadians);
  }

  public double getMaxWristDegrees() {
    return Units.radiansToDegrees(m_MaxWristRadians);
  }

  public double getRotateArmDegrees() {
    return Units.radiansToDegrees(m_RotateArmEncoder.getPosition());
  }

  public double getLinearArmDegrees() {
    return Units.radiansToDegrees(m_LinearArmEncoder.getPosition());
  }

  public double getWristDegrees() {
    return Units.radiansToDegrees(m_WristEncoder.getPosition());
  }

  public void driveRotateArm(double throttle) {
    if (Math.abs(throttle) > 0.05) {
      m_RotateArmPositionRadians += Units.degreesToRadians(throttle);
    }
    setReferencePeriodic();
  }

  public void driveLinearArm(double throttle) {
    if (Math.abs(throttle) > 0.05) {
      m_LinearArmPositionRadians += Units.degreesToRadians(throttle * 10);
    }
    setReferencePeriodic();
  }

  public void driveWrist(double throttle) {
    if (Math.abs(throttle) > 0.05) {
      m_WristPositionRadians += Units.degreesToRadians(throttle * 2);
    }
    setReferencePeriodic();
  }

  public void setReferencePeriodic() {
    m_RotateArmPositionRadians = MathUtil.clamp(m_RotateArmPositionRadians, m_MinRotateArmRadians, m_MaxRotateArmRadians);
    m_LinearArmPositionRadians = MathUtil.clamp(m_LinearArmPositionRadians, m_MinLinearArmRadians, m_MaxLinearArmRadians);
    m_WristPositionRadians = MathUtil.clamp(m_WristPositionRadians, m_MinWristRadians, m_MaxWristRadians);

    m_RotateArmController.setReference(m_RotateArmPositionRadians, ControlType.kPosition);
    m_LinearArmController.setReference(m_LinearArmPositionRadians, ControlType.kPosition);
    m_WristController.setReference(m_WristPositionRadians, ControlType.kPosition);
  }
}
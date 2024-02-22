/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Retractor extends SubsystemBase {
  /*
   * Singleton
   */

  private static Retractor instance = null;

  public static Retractor getInstance() {
    if (instance == null) instance = new Retractor();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 13;

  private static final boolean kInvertMotor = true;
  private static final boolean kInvertEncoder = true;

  private static final int kCurrentLimit = 40;

  private static final double kGearRatio = 360.0;

  private static final float kMinAngle = -15f;
  private static final float kMaxAngle = 160.0f;

  private static final double kP = 0.0065;
  private static final double kI = 0.000;
  private static final double kD = 0.0001;

  private static final double kS = 0.000;
  private static final double kG = 0.0155;
  private static final double kV = 0.000;

  private static final double kMaxVelocityDeg = 120.0;
  private static final double kMaxAccelerationDeg = 90.0;
  private static TrapezoidProfile.Constraints kProfileConstraints =
      new Constraints(kMaxVelocityDeg, kMaxAccelerationDeg);
  private static final double kMaxOutput = 0.2;
  private static final double kMinOutput = -0.2;

  private static final double kToleranceDeg = 2.0;

  private enum RetractorSetpoint {
    kRetracted(160),
    kMating(160),
    kIntake(-15.0);

    public final double angle;

    private RetractorSetpoint(double angleDeg) {
      this.angle = angleDeg;
    }
  }

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;
  private ArmFeedforward feedforwardController;
  private PIDController feedbackController;
  private TrapezoidProfile motionProfile;
  private TrapezoidProfile.State previousReference;
  private TrapezoidProfile.State goalReference;

  private Retractor() {

    this.motor = new CANSparkMax(kMotorPort, kBrushless);
    this.encoder = this.motor.getAbsoluteEncoder(Type.kDutyCycle);
    this.feedforwardController = new ArmFeedforward(kS, kG, kV);
    this.feedbackController = new PIDController(kP, kI, kD);
    this.motionProfile = new TrapezoidProfile(kProfileConstraints);
    this.previousReference = new TrapezoidProfile.State(this.getPositionDeg(), 0);
    this.goalReference = previousReference;

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);

    // this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) kMaxAngle);
    // this.motor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) kMinAngle);
    // this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);
    // this.motor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);

    this.motor.enableSoftLimit(SoftLimitDirection.kForward, false);
    this.motor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  private void configureEncoder() {
    // this.motor.getEncoder().setPositionConversionFactor(kGearRatio);
    // this.motor.getEncoder().setPosition(this.getPositionDeg());

    this.encoder.setInverted(kInvertEncoder);
    this.encoder.setPositionConversionFactor(kGearRatio);
    this.encoder.setVelocityConversionFactor(kGearRatio);
  }

  private void configureController() {
    feedbackController.enableContinuousInput(0.0, 360.0);
    // feedbackController.setTolerance(kToleranceDeg);
    feedbackController.setSetpoint(this.getPositionDeg()); // previousReference.position
  }

  public double getPositionDeg() {
    return MathUtil.clamp(
        this.encoder.getPosition() > 180
            ? this.encoder.getPosition() - 360.0
            : this.encoder.getPosition(),
        kMinAngle,
        kMaxAngle);
  }

  public double getVelocityDeg() {
    return this.encoder.getVelocity();
  }

  private boolean atGoal() {
    return Math.abs(goalReference.position - previousReference.position) < kToleranceDeg;
  }

  private void setGoal(double position) {
    goalReference = new TrapezoidProfile.State(position, 0);
  }

  private boolean atSetpoint() {
    return this.feedbackController.atSetpoint();
  }

  private void setSetpoint(double position) {
    this.feedbackController.setSetpoint(position);
  }

  private double getSetpoint() {
    return this.feedbackController.getSetpoint();
  }

  protected void useOutput() {
    double feedforwardOutput =
        this.feedforwardController.calculate(
            getSetpoint() * Math.PI / 180.0, 0); // setpoint.velocity * Math.PI / 180.0

    SmartDashboard.putNumber("retractor position setpoint mp (deg)", getPositionDeg());
    SmartDashboard.putNumber("retractor velocity setpoint mp (deg)", getVelocityDeg());

    SmartDashboard.putNumber("retractor ff output", feedforwardOutput);

    double feedbackOutput = feedbackController.calculate(this.getPositionDeg());

    SmartDashboard.putNumber("retractor fb output", feedbackOutput);

    double controllerOutput =
        MathUtil.clamp(feedforwardOutput + feedbackOutput, kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("retractor controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  private void doSendables() {
    SmartDashboard.putBoolean("retractor at setpoint", this.atSetpoint());
    SmartDashboard.putNumber("retractor position (deg)", this.getPositionDeg());
    SmartDashboard.putNumber("retractor velocity (deg)", this.getVelocityDeg());
  }

  @Override
  public void periodic() {
    // motion profile version
    // previousReference = motionProfile.calculate(0.02, previousReference, goalReference);
    // useOutput(previousReference);

    useOutput();

    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("retractor kp", SmartDashboard.getNumber("retractor kp", kP));
    SmartDashboard.putNumber("retractor ki", SmartDashboard.getNumber("retractor ki", kI));
    SmartDashboard.putNumber("retractor kd", SmartDashboard.getNumber("retractor kd", kD));

    SmartDashboard.putNumber("retractor ks", SmartDashboard.getNumber("retractor ks", kS));
    SmartDashboard.putNumber("retractor kv", SmartDashboard.getNumber("retractor kv", kV));
    SmartDashboard.putNumber("retractor kg", SmartDashboard.getNumber("retractor kg", kG));

    SmartDashboard.putNumber(
        "retractor max vel (deg)",
        SmartDashboard.getNumber("retractor max vel (deg)", kMaxVelocityDeg));
    SmartDashboard.putNumber(
        "retractor max acc (deg)",
        SmartDashboard.getNumber("retractor max acc (deg)", kMaxAccelerationDeg));

    SmartDashboard.putNumber("retractor target position (deg)", this.getPositionDeg());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("retractor kp", kP);
    double tunedI = SmartDashboard.getNumber("retractor ki", kI);
    double tunedD = SmartDashboard.getNumber("retractor kd", kD);

    feedbackController.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("retractor ks", kS);
    double tunedV = SmartDashboard.getNumber("retractor kv", kV);
    double tunedG = SmartDashboard.getNumber("retractor kg", kG);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double tunedMaxVel = SmartDashboard.getNumber("retractor max vel (deg)", kMaxVelocityDeg);
    double tunedMaxAcc = SmartDashboard.getNumber("retractor max acc (deg)", kMaxAccelerationDeg);

    this.motionProfile = new TrapezoidProfile(new Constraints(tunedMaxVel, tunedMaxAcc));

    this.setSetpoint(
        MathUtil.clamp(
            SmartDashboard.getNumber("retractor target position (deg)", this.getPositionDeg()),
            kMinAngle,
            kMaxAngle));
  }

  private Command moveToAngle(DoubleSupplier angleDegSupplier) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () ->
                this.setSetpoint(
                    MathUtil.clamp(angleDegSupplier.getAsDouble(), kMinAngle, kMaxAngle))),
        this.run(
                () ->
                    this.setSetpoint(
                        MathUtil.clamp(angleDegSupplier.getAsDouble(), kMinAngle, kMaxAngle)))
            .until(this::atSetpoint));
  }

  private Command moveToAngle(double angleDeg) {
    return this.moveToAngle(() -> angleDeg);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(RetractorSetpoint.kRetracted.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(RetractorSetpoint.kMating.angle);
  }

  public Command moveToIntake() {
    return this.moveToAngle(RetractorSetpoint.kIntake.angle);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}

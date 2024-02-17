/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.launcher;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

import com.revrobotics.*;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Pivot extends SubsystemBase {
  /*
   * Singleton
   */

  private static Pivot instance = null;

  public static Pivot getInstance() {
    if (instance == null) instance = new Pivot();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 15;

  private static final boolean kInvertMotor = false;
  private static final boolean kInvertEncoder = false;
  private static final int kCurrentLimit = 40;

  private static final double kGearRatio = 360.0;

  private static final float kMinAngle = 0.0f;
  private static final float kMaxAngle = 90.0f;

  private static final double kP = 0.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final double kS = 0.0;
  private static final double kG = 0.0;
  private static final double kV = 0.0;

  private static final double kMaxVelocityDeg = 40;
  private static final double kMaxAccelerationDeg = 20;
  private static TrapezoidProfile.Constraints kProfileConstraints = new Constraints(kMaxVelocityDeg, kMaxAccelerationDeg);


  private static final double kToleranceDeg = 2.5;

  private enum PivotSetpoint {
    kRetracted(0.0),
    kMating(0.0),
    kAmp(0.0);

    public final double angle;

    private PivotSetpoint(double angle) {
      this.angle = angle;
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

  private Pivot() {
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
    feedbackController.setSetpoint(previousReference.position);
  }
  
  public double getPositionDeg() {
    // return this.encoder.getPosition() > 180 ? this.encoder.getPosition() - 360.0 : this.encoder.getPosition();
    return this.encoder.getPosition();
  }

  public double getVelocityDeg() {
    return this.encoder.getVelocity();
  }

  private boolean atGoal(){
    if(Math.abs(goalReference.position - previousReference.position) < kToleranceDeg){
      return true;
    } else{
      return false;
    }
  }

  private void setGoal(double position){
    goalReference = new TrapezoidProfile.State(position, 0);
  }

  protected void useOutput(TrapezoidProfile.State setpoint) {
    double feedforwardOutput =
        this.feedforwardController.calculate(setpoint.position * Math.PI / 180.0, setpoint.velocity * Math.PI / 180.0);

    SmartDashboard.putNumber("pivot position setpoint mp (deg)", setpoint.position);
    SmartDashboard.putNumber("pivot velocity setpoint mp (deg)", setpoint.velocity);

    SmartDashboard.putNumber("pivot ff output", feedforwardOutput);

    double feedbackOutput = feedbackController.calculate(this.getPositionDeg(), setpoint.position);

    SmartDashboard.putNumber("pivot fb output", feedbackOutput);

    double controllerOutput = feedforwardOutput + feedbackOutput;

    SmartDashboard.putNumber("pivot controller output", controllerOutput);

    this.motor.set(controllerOutput);
  }

  public void doSendables() {
    SmartDashboard.putBoolean("pivot at goal", this.atGoal());
    SmartDashboard.putNumber("pivot position (deg)", this.getPositionDeg());
  }

  @Override
  public void periodic() {
    // TrapezoidProfile profile = new TrapezoidProfile(kProfileConstraints, goalReference, previousReference);
    // previousReference = profile.calculate(0.02);
    previousReference = motionProfile.calculate(0.02, previousReference, goalReference);

    useOutput(previousReference);
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("pivot kp", SmartDashboard.getNumber("pivot kp", kP));
    SmartDashboard.putNumber("pivot ki", SmartDashboard.getNumber("pivot ki", kI));
    SmartDashboard.putNumber("pivot kd", SmartDashboard.getNumber("pivot kd", kD));

    SmartDashboard.putNumber("pivot ks", SmartDashboard.getNumber("pivot ks", kS));
    SmartDashboard.putNumber("pivot kg", SmartDashboard.getNumber("pivot kg", kG));
    SmartDashboard.putNumber("pivot kv", SmartDashboard.getNumber("pivot kv", kV));

    SmartDashboard.putNumber("pivot max vel (deg)", SmartDashboard.getNumber("pivot max vel (deg)", kMaxVelocityDeg));
    SmartDashboard.putNumber("pivot max acc (deg)", SmartDashboard.getNumber("pivot max acc (deg)", kMaxAccelerationDeg));

    SmartDashboard.putNumber("pivot target position (deg)", this.getPositionDeg());
  }

  private void tune() {
    double tunedP = SmartDashboard.getNumber("pivot kp", kP);
    double tunedI = SmartDashboard.getNumber("pivot ki", kI);
    double tunedD = SmartDashboard.getNumber("pivot kd", kD);

    feedbackController.setPID(tunedP, tunedI, tunedD);

    double tunedS = SmartDashboard.getNumber("pivot ks", kS);
    double tunedG = SmartDashboard.getNumber("pivot kg", kG);
    double tunedV = SmartDashboard.getNumber("pivot kv", kV);

    this.feedforwardController = new ArmFeedforward(tunedS, tunedG, tunedV);

    double tunedMaxVel = SmartDashboard.getNumber("pivot max vel (deg)", kMaxVelocityDeg);
    double tunedMaxAcc = SmartDashboard.getNumber("pivot max acc (deg)", kMaxAccelerationDeg);

    this.motionProfile = new TrapezoidProfile(new Constraints(tunedMaxVel, tunedMaxAcc));

    this.setGoal(MathUtil.clamp(SmartDashboard.getNumber("pivot target position (deg)", this.getPositionDeg()), kMinAngle, kMaxAngle));
  }

  private Command moveToAngle(DoubleSupplier angleDegSupplier) {
    return run(() -> this.setGoal(MathUtil.clamp(angleDegSupplier.getAsDouble(), kMinAngle, kMaxAngle))).until(this::atGoal);
  }

  private Command moveToAngle(double angleDeg) {
    return this.moveToAngle(() -> angleDeg);
  }

  public Command moveToRetracted() {
    return this.moveToAngle(PivotSetpoint.kRetracted.angle);
  }

  public Command moveToMating() {
    return this.moveToAngle(PivotSetpoint.kMating.angle);
  }

  public Command aimAtAmp() {
    return this.moveToAngle(PivotSetpoint.kAmp.angle);
  }

  public Command aimAtSpeaker(DoubleSupplier angleDegSupplier) {
    return this.moveToAngle(angleDegSupplier);
  }

  public Command tuneControllers() {
    this.initTuning();

    return run(this::tune);
  }
}

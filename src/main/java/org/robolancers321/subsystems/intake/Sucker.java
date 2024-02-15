/* (C) Robolancers 2024 */
package org.robolancers321.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sucker extends SubsystemBase {
  /*
   * Singleton
   */

  private static Sucker instance = null;

  public static Sucker getInstance() {
    if (instance == null) instance = new Sucker();

    return instance;
  }

  /*
   * Constants
   */

  private static final int kMotorPort = 14;

  private static final boolean kInvertMotor = false;
  private static final int kCurrentLimit = 20;

  private static final double kFF = 0.00;

  private static final double kInRPM = -2000;
  private static final double kOutRPM = 1000;

  /*
   * Implementation
   */

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkPIDController controller;

  private Sucker() {
    this.motor = new CANSparkMax(kMotorPort, MotorType.kBrushless);
    this.encoder = this.motor.getEncoder();
    this.controller = this.motor.getPIDController();

    this.configureMotor();
    this.configureEncoder();
    this.configureController();
    this.motor.burnFlash();
  }

  private void configureMotor() {
    this.motor.setInverted(kInvertMotor);
    this.motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.motor.setSmartCurrentLimit(kCurrentLimit);
    this.motor.enableVoltageCompensation(12);
  }

  private void configureEncoder() {
    this.encoder.setVelocityConversionFactor(1.0);
  }

  private void configureController() {
    this.controller.setP(0.0);
    this.controller.setI(0.0);
    this.controller.setD(0.0);
    this.controller.setFF(kFF);
  }

  public double getVelocityRPM() {
    return this.encoder.getVelocity();
  }

  private void useController(double desiredRPM) {
    this.controller.setReference(desiredRPM, ControlType.kVelocity);
  }

  private void doSendables() {
    SmartDashboard.putNumber("sucker rpm", this.getVelocityRPM());
  }

  @Override
  public void periodic() {
    this.doSendables();
  }

  private void initTuning() {
    SmartDashboard.putNumber("sucker kff", SmartDashboard.getNumber("sucker kff", kFF));
    SmartDashboard.putNumber("sucker target rpm", 0.0);
  }

  private void tune() {
    double tunedFF = SmartDashboard.getNumber("sucker kff", kFF);

    this.controller.setFF(tunedFF);

    double targetRPM = SmartDashboard.getNumber("sucker target rpm", 0.0);

    this.useController(targetRPM);
  }

  public Command in() {
    return run(() -> this.useController(kInRPM)).finallyDo(() -> this.useController(0.0));
  }

  public Command out() {
    return run(() -> this.useController(kOutRPM)).finallyDo(() -> this.useController(0.0));
  }

  public Command tuneController() {
    initTuning();

    return run(this::tune);
  }
}
/* (C) Robolancers 2024 */
package org.robolancers321;

public final class Constants {
  public static final class ClimberConstants {
    public static final int kLeftClimberPort = 0;
    public static final int kLeftLimitSwitchPort = 0;
    public static final boolean kLeftClimberInverted = false;

    public static final int kRightClimberPort = 0;
    public static final int kRightLimitSwitchPort = 1;
    public static final boolean kRightClimberInverted = false;

    public static final int kCurrentLimit = 40;
    public static final float kMaxSoftLimit = 1;
    public static final float kMinSoftLimit = 0;
    public static final double kMetersPerRot = 1;
    public static final double kRPMToMPS = kMetersPerRot / 60.0;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static final double kErrorTolerance = 0.1;

    // used to zero the climber at a safe speed
    public static final double kDownwardZeroingSpeed = -0.2;

    public enum ClimberSetpoint {
      kRetracted(0),
      kTrap(0),
      kFullExtend(0);

      public final double position;

      ClimberSetpoint(double position) {
        this.position = position;
      }
    }
  }
}

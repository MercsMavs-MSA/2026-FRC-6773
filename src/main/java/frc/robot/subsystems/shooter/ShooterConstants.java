package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.Constants;

public class ShooterConstants {
  // TODO:
  public static final Rotation2d turretMaxLimit = Rotation2d.fromRotations(0.653);
  public static final Rotation2d turretMinLimit = Rotation2d.fromRotations(-0.343);

  public static final Rotation2d turretPositionTolerance = new Rotation2d(Math.toRadians(0.5));

  public static Transform3d robotToTurret = new Transform3d(-0.14, 0.13, 0.55, Rotation3d.kZero);

  public static AngularVelocity flywheelThreshold = RPM.of(2300);

  public record ShooterFlywheelHardware(
      int flyWheelMotorLeftId, int flyWheelMotorRightId, double gearing) {}

  public record ShooterHoodHardware(int hoodMotorId, double gearing) {}

  public record ShooterTurretHardware(int turretMotorId, int cancoderID, double gearing) {}

  public record TurretGains(double p, double i, double d, double s, double v, double a) {}

  public record FlywheelGains(double p, double i, double d, double s, double v, double a) {}

  public record HoodGains(double p, double i, double d, double s, double v, double a) {}

  public record FlywheelMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record TurretMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record HoodMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public record SimulationConfiguration(DCMotor motorType, double measurementStdDevs) {}

  public static final ShooterFlywheelHardware flywheelHardware =
      new ShooterFlywheelHardware(56, 57, 6000d / 4800d);

  public static final ShooterHoodHardware hoodHardware = new ShooterHoodHardware(55, 163.28);

  public static final ShooterTurretHardware turretHardware = new ShooterTurretHardware(53, 54, 10d);

  public static final TurretMotorConfiguration turretConfigs =
      new TurretMotorConfiguration(false, true, true, 60, 50, 6, -6, NeutralModeValue.Brake);
  public static final FlywheelMotorConfiguration flywheelConfigs =
      new FlywheelMotorConfiguration(true, true, true, 80, 60, 12, -12, NeutralModeValue.Coast);
  public static final HoodMotorConfiguration hoodConfigs =
      new HoodMotorConfiguration(true, true, true, 60, 30, 12, -12, NeutralModeValue.Brake);

  public static final SimulationConfiguration shooterTurretSimConfig =
      new SimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);
  public static final SimulationConfiguration shooterFlywheelSimConfig =
      new SimulationConfiguration(DCMotor.getKrakenX60(2), 0.002);
  public static final SimulationConfiguration shooterHoodSimConfig =
      new SimulationConfiguration(DCMotor.getKrakenX44(1), 0.002);

  public static final FlywheelGains flywheelGains =
      new FlywheelGains(0.2, 0.0, 0.0, 0.1, 0.126, 0.0);
  public static final HoodGains hoodGains = new HoodGains(70, 0, 0, 0.1, 0, 0);
  public static final TurretGains turretGains =
      switch (Constants.currentMode) {
        case REAL -> new TurretGains(35, 0.0, 0.0, 0.5, 5.3, 0.0);
        case SIM -> new TurretGains(5, 0.0, 0.01, 0.0, 5.0, 0.0);
        default -> new TurretGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final double wrapAroundDegreesThreshold = 20;
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterTurretHardware;
import frc.robot.subsystems.shooter.ShooterConstants.TurretGains;
import frc.robot.subsystems.shooter.ShooterConstants.TurretMotorConfiguration;

public class ShooterTurretIOTalonFX implements ShooterTurretIO {
  private final TalonFX turretMotor;
  private final CANcoder turretCANcoder;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
  private CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

  // Motor data we wish to log
  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSec;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  private NeutralModeValue currentMode = NeutralModeValue.Brake;

  // Control modes
  private final VoltageOut kVoltageControl = new VoltageOut(0.0);
  private final PositionVoltage kPositionControl = new PositionVoltage(0.0);

  public ShooterTurretIOTalonFX(
      String canbus,
      ShooterTurretHardware hardware,
      TurretMotorConfiguration configuration,
      TurretGains gains,
      double statusSignalUpdateFrequency) {

    turretMotor = new TalonFX(hardware.turretMotorId(), canbus);
    turretCANcoder = new CANcoder(hardware.cancoderID(), canbus);

    canCoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfiguration.MagnetSensor.MagnetOffset =
        -0.248046875; // TODO: THIS IS THE MAGNET OFFSET DO NOT CHANGE THIS
    // PLEAASEE
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    turretCANcoder.getConfigurator().apply(canCoderConfiguration);

    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kS = gains.s();

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();

    motorConfiguration.withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ShooterConstants.turretMaxLimit.getRotations())
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ShooterConstants.turretMinLimit.getRotations()));

    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted =
        configuration.invert()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    // Rotor sensor is the built-in sensor
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfiguration.Feedback.FeedbackRemoteSensorID = turretCANcoder.getDeviceID();

    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
    motorConfiguration.Feedback.RotorToSensorRatio = 5.0;

    motorConfiguration.ClosedLoopGeneral.ContinuousWrap = false;

    // Get status signals from the motor controller
    positionRotations = turretMotor.getPosition();
    velocityRotationsPerSec = turretMotor.getVelocity();
    appliedVolts = turretMotor.getMotorVoltage();
    supplyCurrentAmps = turretMotor.getSupplyCurrent();
    statorCurrentAmps = turretMotor.getStatorCurrent();
    temperatureCelsius = turretMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        statusSignalUpdateFrequency,
        positionRotations,
        velocityRotationsPerSec,
        appliedVolts,
        supplyCurrentAmps,
        statorCurrentAmps,
        temperatureCelsius);

    // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // are not using to simply not be sent over the CANBus
    turretMotor.optimizeBusUtilization(0.0, 1.0);
    turretMotor.getConfigurator().apply(motorConfiguration, 1);
  }

  public ShooterTurretIOTalonFX(
      ShooterTurretHardware hardware,
      TurretMotorConfiguration configuration,
      TurretGains gains,
      double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", hardware, configuration, gains, statusSignalUpdateFrequency);
  }

  @Override
  public void updateInputs(ShooterTurretIOInputs inputs) {
    inputs.isMotorConnected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRotationsPerSec,
                appliedVolts,
                supplyCurrentAmps,
                supplyCurrentAmps,
                statorCurrentAmps,
                temperatureCelsius)
            .isOK();

    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRotPerSec = velocityRotationsPerSec.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setControl(kVoltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(Rotation2d goalPosition) {
    turretMotor.setControl(kPositionControl.withPosition(goalPosition.getRotations()).withSlot(0));
  }

  public void setNeutralMode(NeutralModeValue value) {
    turretMotor.setNeutralMode(value);
  }

  @Override
  public void stop() {
    turretMotor.setControl(new NeutralOut());
  }

  @Override
  public void resetPosition() {
    turretMotor.setPosition(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double v, double a) {
    var slotConfiguration = new Slot0Configs();

    slotConfiguration.kP = p;
    slotConfiguration.kI = i;
    slotConfiguration.kD = d;
    slotConfiguration.kS = s;
    slotConfiguration.kV = v;
    slotConfiguration.kA = a;

    turretMotor.getConfigurator().apply((slotConfiguration));
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    NeutralModeValue newMode = enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    if (currentMode != newMode) {
      turretMotor.setNeutralMode(newMode);
      currentMode = newMode;
    }
  }

  @Override
  public void setTurretSetpoint(Angle position, AngularVelocity velocity) {
    turretMotor.setControl(kPositionControl.withPosition(position).withVelocity(velocity));
  }

  @Override
  public Angle getTurretPosition() {
    // turretMotor.getIs
    return turretMotor.getPosition().getValue();
  }
}

// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.annotation.AutoLogMavs;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.FieldConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.ZoneUtil;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  public enum DriveState {
    IDLE,
    SHOOTING,
    BUMP,
    DRIVING,
    ALIGN
  }

  public enum Zone {
    NEUTRAL_RIGHT,
    NEUTRAL_LEFT,
    ALLIANCE
  }

  public DriveState driveState = DriveState.IDLE;

  public double speedCap = Double.MAX_VALUE;

  public double lastVelX;
  public double lastVelY;
  public Pose2d lastPose;
  public Timer timer;
  public double accelerationX;
  public double accelerationY;
  public AngularVelocity lastOmega;
  public AngularAcceleration alpha;

  private static final double kAccelFilterAlpha = 0.15;

  private static final double rookieSpeedCap = 3.0; // m/s

  private Trigger bumpTrigger;
  private BooleanSupplier isShootingActive = () -> false;
  private final BooleanEntry shootingSlowdownEnabled =
      NetworkTableInstance.getDefault()
          .getTable("Drive")
          .getBooleanTopic("ShootingSlowdownEnabled")
          .getEntry(true);
  private final BooleanEntry rookieModeEnabled =
      NetworkTableInstance.getDefault()
          .getTable("Drive")
          .getBooleanTopic("RookieModeEnabled")
          .getEntry(false);

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = DriveConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
              Math.hypot(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
              Math.hypot(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              DriveConstants.FrontLeft.WheelRadius,
              DriveConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(DriveConstants.FrontLeft.DriveMotorGearRatio),
              DriveConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, DriveConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, DriveConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, DriveConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, DriveConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    shootingSlowdownEnabled.set(true);
    rookieModeEnabled.set(false);

    bumpTrigger =
        ZoneUtil.BUMP_ZONES.willContain(this::getPose, this::getFieldVelocity, Seconds.of(0.3));
    bumpTrigger.onTrue(Commands.runOnce(() -> setDriveState(DriveState.BUMP)));
    bumpTrigger.onFalse(
        Commands.runOnce(
            () ->
                setDriveState(
                    isShootingActive.getAsBoolean() ? DriveState.SHOOTING : DriveState.DRIVING)));
    bumpTrigger.debounce(0.5);
    timer = new Timer();
    timer.start();
    lastPose = getPose();
    lastVelX = 0;
    lastVelY = 0;
    lastOmega = AngularVelocity.ofBaseUnits(0, RadiansPerSecond);
    alpha = AngularAcceleration.ofBaseUnits(0, RadiansPerSecondPerSecond);
  }

  @Override
  public void periodic() {

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      switch (driveState) {
        case IDLE:
          speedCap = 0;
          break;
        case DRIVING:
          speedCap = Double.MAX_VALUE;
          break;
        case SHOOTING:
          speedCap = Double.MAX_VALUE;
          break;
        case BUMP:
          speedCap = 2.5; // m / s
          break;
        case ALIGN:
          speedCap = 3.0; // m / s
          break;
        default:
          speedCap = Double.MAX_VALUE;
          break;
      }

      Logger.recordOutput("States/DriveState", driveState);
      Logger.recordOutput("Speed Cap", speedCap);

      Logger.recordOutput(
          "Drive/RobotSpeed",
          Math.sqrt(
              Math.pow(getFieldVelocity().vxMetersPerSecond, 2)
                  + Math.pow(getFieldVelocity().vyMetersPerSecond, 2)));
      Logger.recordOutput(
          "Drive/RobotAcceleration",
          Math.sqrt(Math.pow(accelerationX, 2) + Math.pow(accelerationY, 2)));
    }

    // Calculate acceleration outside the odometry sample loop to avoid near-zero dT
    if (timer.get() != 0) {
      double dT = timer.get();
      double distanceX = getPose().getX() - lastPose.getX();
      double distanceY = getPose().getY() - lastPose.getY();
      double thisVelX = distanceX / dT;
      thisVelX = getFieldVelocity().vxMetersPerSecond;
      double thisVelY = distanceY / dT;
      thisVelY = getFieldVelocity().vyMetersPerSecond;
      double rawAccelX = (thisVelX - lastVelX) / dT;
      double rawAccelY = (thisVelY - lastVelY) / dT;

      // Low-pass filter to reduce jitter
      accelerationX = kAccelFilterAlpha * rawAccelX + (1 - kAccelFilterAlpha) * accelerationX;
      accelerationY = kAccelFilterAlpha * rawAccelY + (1 - kAccelFilterAlpha) * accelerationY;
      // accelerationX = rawAccelX;
      // accelerationY = rawAccelY;

      lastVelX = thisVelX;
      lastVelY = thisVelY;

      AngularVelocity omega =
          AngularVelocity.ofBaseUnits(getFieldVelocity().omegaRadiansPerSecond, RadiansPerSecond);

      double rawAlpha = (getFieldVelocity().omegaRadiansPerSecond - lastOmega.magnitude()) / dT;

      alpha = AngularAcceleration.ofBaseUnits(rawAlpha, RadiansPerSecondPerSecond);
      lastOmega = omega;
      lastPose = getPose();
      timer.reset();
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void setDriveState(DriveState state) {
    this.driveState = state;
  }

  public DriveState getDriveState() {
    return driveState;
  }

  public void setShootingActiveSupplier(BooleanSupplier supplier) {
    this.isShootingActive = supplier;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // double effectiveCap =
    //     Math.min(speedCap, rookieModeEnabled.get() ? rookieSpeedCap : Double.MAX_VALUE);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, Math.min(speedCap, DriveConstants.kSpeedAt12Volts.in(MetersPerSecond)));

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
  }

  /** Returns the current odometry pose. */
  @AutoLogMavs(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  @AutoLogMavs(key = "HELLO")
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  // Returns a Pose2d containing linear accelerations in meters and angular acceleration in radians
  @AutoLogOutput(key = "Drive/Accels")
  public Pose2d getAccelComponents() {
    return new Pose2d(
        accelerationX, accelerationY, Rotation2d.fromRadians(alpha.abs(DegreesPerSecondPerSecond)));
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public Rotation2d interpolateAngle(Pose2d pose1, Pose2d pose2) {
    return pose2
        .getTranslation()
        .minus(pose1.getTranslation()) // Vector from point1 to point2
        .getAngle()
        .rotateBy(Rotation2d.k180deg);
  }

  public boolean checkInAllianceZone(Pose2d robotPose) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return robotPose.getX() <= FieldConstants.LinesVertical.allianceZone;
    } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return robotPose.getX() >= AllianceFlipUtil.applyX(FieldConstants.LinesVertical.allianceZone);
    }
    return false;
  }

  public Zone returnZone(Pose2d robotPose) {
    if (checkInAllianceZone(robotPose)) {
      return Zone.ALLIANCE;
    }

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      if (robotPose.getY() < FieldConstants.Hub.innerCenterPoint.getY()) {
        return Zone.NEUTRAL_RIGHT;
      }
      return Zone.NEUTRAL_LEFT;
    }

    if (robotPose.getY() > FieldConstants.Hub.innerCenterPoint.getY()) {
      return Zone.NEUTRAL_RIGHT;
    }
    return Zone.NEUTRAL_LEFT;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(DriveConstants.FrontLeft.LocationX, DriveConstants.FrontLeft.LocationY),
      new Translation2d(DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY),
      new Translation2d(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
      new Translation2d(DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY)
    };
  }

  public Rotation2d getFieldSpeedAngle() {
    ChassisSpeeds speeds = this.getFieldVelocity();
    if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.5)
      return this.getRotation();
    return new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond));
  }
}

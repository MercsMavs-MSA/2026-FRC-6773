// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.annotation.AutoLogMavsItem;
import frc.robot.commands.AutonCommands;
// import frc.robot.RobotManager.IntakeManagerState;
// import frc.robot.RobotManager.RobotScoringState;
// import frc.robot.commands.AutonCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.TeleopCommands;
// import frc.robot.commands.TeleopCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.indexer.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerKickerIOSim;
import frc.robot.subsystems.indexer.IndexerKickerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSpindexerIOSim;
import frc.robot.subsystems.indexer.IndexerSpindexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakePivotIOSim;
import frc.robot.subsystems.intake.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterFlywheelIOSim;
import frc.robot.subsystems.shooter.ShooterFlywheelIOTalonFX;
import frc.robot.subsystems.shooter.ShooterHoodIOSim;
import frc.robot.subsystems.shooter.ShooterHoodIOTalonFX;
import frc.robot.subsystems.shooter.ShooterTurretCalculator;
import frc.robot.subsystems.shooter.ShooterTurretIOSim;
import frc.robot.subsystems.shooter.ShooterTurretIOTalonFX;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.Transfer.TransferState;
import frc.robot.subsystems.transfer.TransferConstants;
import frc.robot.subsystems.transfer.TransferIOSim;
import frc.robot.subsystems.transfer.TransferIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.FuelSim;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @AutoLogMavsItem public final Drive drive;
  @AutoLogMavsItem public Vision vision;
  @AutoLogMavsItem public final Indexer indexer;
  @AutoLogMavsItem public final Transfer transfer;
  @AutoLogMavsItem public final Intake intake;
  @AutoLogMavsItem public final Shooter shooter;
  // private final Climb climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController opController = new CommandXboxController(1);

  private final AutonCommands autonCommands;
  private final TeleopCommands teleopCommands;

  // private RobotManager manager;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private Field2d field = new Field2d();
  private Field2d trajField = new Field2d();

  private Trigger rumbleTrigger;

  // private PowerDistribution pdh;

  public FuelSim fuelSim;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.FrontLeft),
                new ModuleIOTalonFX(DriveConstants.FrontRight),
                new ModuleIOTalonFX(DriveConstants.BackLeft),
                new ModuleIOTalonFX(DriveConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOLimelight(camera2Name, drive::getRotation),
                new VisionIOLimelight(camera3Name, drive::getRotation));
        shooter =
            new Shooter(
                new ShooterFlywheelIOTalonFX(
                    ShooterConstants.flywheelHardware,
                    ShooterConstants.flywheelConfigs,
                    ShooterConstants.flywheelGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                new ShooterTurretIOTalonFX(
                    ShooterConstants.turretHardware,
                    ShooterConstants.turretConfigs,
                    ShooterConstants.turretGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                new ShooterHoodIOTalonFX(
                    ShooterConstants.hoodHardware,
                    ShooterConstants.hoodConfigs,
                    ShooterConstants.hoodGains,
                    ShooterConstants.kStatusSignalUpdateFrequencyHz),
                drive::getPose,
                drive::getFieldVelocity);
        intake =
            new Intake(
                new IntakeRollerIOTalonFX(
                    IntakeConstants.rollerHardware,
                    IntakeConstants.kRollerMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz),
                new IntakePivotIOTalonFX(
                    IntakeConstants.pivotHardware,
                    IntakeConstants.pivotGains,
                    IntakeConstants.kPivotMotorConfiguration,
                    IntakeConstants.kStatusSignalUpdateFrequencyHz),
                drive::getPose,
                drive::getFieldVelocity,
                () -> controller.leftTrigger().getAsBoolean());
        indexer =
            new Indexer(
                new IndexerSpindexerIOTalonFX(
                    IndexerConstants.spindexerHardware,
                    IndexerConstants.spindexerGains,
                    IndexerConstants.spindexerTalonFXConfiguration,
                    IndexerConstants.statusSignalUpdateFrequencyHz),
                new IndexerKickerIOTalonFX(
                    IndexerConstants.kickerHardware,
                    IndexerConstants.kickerGains,
                    IndexerConstants.kickerTalonFXConfiguration,
                    IndexerConstants.statusSignalUpdateFrequencyHz),
                shooter.wrapAroundSupplier());
        transfer =
            new Transfer(
                new TransferIOTalonFX(
                    TransferConstants.transferHardware,
                    TransferConstants.transferGains,
                    TransferConstants.transferTalonFXConfiguration,
                    TransferConstants.statusSignalUpdateFrequencyHz),
                shooter.wrapAroundSupplier());
        // pdh = new PowerDistribution(1, ModuleType.kRev);
        // climber =
        // new Climb(
        // new ClimbIOTalonFX(
        // ClimbConstants.climbHardware,
        // ClimbConstants.climbTalonFXConfiguration,
        // ClimbConstants.statusSignalUpdateFrequencyHz),
        // getClimbAdjustmentDoubleSupplier());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(DriveConstants.FrontLeft),
                new ModuleIOSim(DriveConstants.FrontRight),
                new ModuleIOSim(DriveConstants.BackLeft),
                new ModuleIOSim(DriveConstants.BackRight));

        shooter =
            new Shooter(
                new ShooterFlywheelIOSim(
                    0.02,
                    ShooterConstants.flywheelHardware,
                    ShooterConstants.shooterFlywheelSimConfig),
                new ShooterTurretIOSim(
                    0.02, ShooterConstants.turretHardware, ShooterConstants.shooterTurretSimConfig),
                new ShooterHoodIOSim(
                    0.02, ShooterConstants.hoodHardware, ShooterConstants.shooterHoodSimConfig),
                drive::getPose,
                drive::getFieldVelocity);
        intake =
            new Intake(
                new IntakeRollerIOSim(
                    0.02,
                    IntakeConstants.rollerHardware,
                    IntakeConstants.rollerSimulationConfiguration),
                new IntakePivotIOSim(
                    0.02,
                    IntakeConstants.pivotHardware,
                    IntakeConstants.pivotSimulationConfiguration),
                drive::getPose,
                drive::getFieldVelocity,
                () -> controller.leftTrigger().getAsBoolean());
        indexer =
            new Indexer(
                new IndexerSpindexerIOSim(
                    0.02,
                    IndexerConstants.spindexerHardware,
                    IndexerConstants.spindexerSimulationConfiguration),
                new IndexerKickerIOSim(
                    0.02,
                    IndexerConstants.kickerHardware,
                    IndexerConstants.kickerSimulationConfiguration),
                () -> {
                  return true;
                });
        transfer =
            new Transfer(
                new TransferIOSim(
                    0.02,
                    TransferConstants.transferHardware,
                    TransferConstants.transferSimulationConfiguration),
                () -> {
                  return true;
                });
        // pdh = new PowerDistribution();
        // climber =
        // new Climb(
        // new ClimbIOSim(
        // 0.02,
        // ClimbConstants.climbHardware,
        // ClimbConstants.climbSimulationConfiguration),
        // getClimbAdjustmentDoubleSupplier());

        fuelSim = new FuelSim(""); // creates a new fuelSim of FuelSim
        fuelSim.spawnStartingFuel(); // spawns fuel in the depots and neutral zone

        // Register a robot for collision with fuel
        fuelSim.registerRobot(
            27, // from left to right in meters
            27, // from front to back in meters
            Meters.convertFrom(7.5, Inches), // from floor to top of bumpers in meters
            drive::getPose, // Supplier<Pose2d> of robot pose
            drive::getFieldVelocity); // Supplier<ChassisSpeeds> of field-centric chassis speeds
        // Register an intake to remove fuel from the field as a rectangular bounding
        // box
        fuelSim.registerIntake(
            Meters.convertFrom(11, Inches),
            Meters.convertFrom(25, Inches), // robot-centric coordinates for bounding box in
            // meters
            Meters.convertFrom(-11, Inches),
            Meters.convertFrom(11, Inches),
            () -> {
              return (intake.getIntakeState() == IntakeState.INTAKING) && fuelSim.fuelCount() > 318;
            });

        fuelSim.registerOuttake(
            Meters.convertFrom(24, Inches),
            Meters.convertFrom(25, Inches), // robot-centric coordinates for bounding box in
            // meters
            Meters.convertFrom(-11, Inches),
            Meters.convertFrom(11, Inches),
            () -> {
              return (intake.getIntakeState() == IntakeState.OUTTAKING)
                  && fuelSim.fuelCount() < 348;
            },
            15,
            MetersPerSecond.of(2));

        fuelSim.registerLauncher(
            () -> {
              return shooter.shooterState.name().startsWith("SHOOT")
                  && shooter.isFlywheelAtThreshold()
                  && fuelSim.fuelCount() < 408;
            },
            () ->
                ShooterTurretCalculator.angularToLinearVelocity(
                    shooter.getFlywheelVelocities()[0], Inches.of(2)),
            () -> shooter.getHoodPosition().getMeasure(),
            () -> Rotations.of(shooter.getTurretPosition()).plus(Degrees.of(90)),
            Meters.of(ShooterConstants.robotToTurret.getTranslation().getZ()),
            0);

        // (optional) BooleanSupplier for whether the intake should be active at a given
        // moment); // (optional) Runnable called whenever a fuel is intaked

        // fuelSim.setSubticks(); // sets the number of physics iterations to perform
        // per 20ms
        // loop.
        // Default = 5

        fuelSim.start(); // enables the simulation to run (updateSim must still be called
        // periodically)
        // fuelSim.stop(); // stops the simulation running (updateSim will do nothing
        // until start
        // is
        // called again)

        fuelSim.enableAirResistance(); // an additional drag force will be applied to fuel in
        // physics
        // update step

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                // RobotState.getInstance()::addVisionObservation,
                drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        intake = new Intake(null, null, null, null, null);
        indexer = new Indexer(null, null, null);
        transfer = new Transfer(null, null);
        shooter = new Shooter(null, null, null, null, null);

        // climber = new Climb(null, null);
        break;
    }
    // manager = new RobotManager(drive, intake, indexer, transfer, shooter);
    teleopCommands =
        new TeleopCommands(
            drive,
            intake,
            indexer,
            transfer,
            shooter,
            controller.rightTrigger().or(opController.rightBumper())
            // , climber
            );
    autonCommands =
        new AutonCommands(
            drive, intake, indexer, transfer, shooter
            // , climber
            );
    rumbleTrigger =
        new Trigger(
            () -> {
              Pose2d accels = drive.getAccelComponents();
              if (accels.getTranslation().getNorm() > 6 && !DriverStation.isAutonomousEnabled()) {
                return true;
              }
              // if (accels.getRotation().getDegrees() > ) {
              // return true;
              // }
              return false;
            });

    rumbleTrigger.onTrue(
        Commands.runOnce(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0.5);
            }));
    rumbleTrigger.onFalse(
        Commands.runOnce(
            () -> {
              controller.setRumble(RumbleType.kBothRumble, 0);
            }));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // autoChooser.addOption("LEFT_45",
    // autonCommands.getAutonomousSequence("LEFT_45"));
    // autoChooser.addOption("RIGHT_45",
    // autonCommands.getAutonomousSequence("RIGHT_45"));
    // autoChooser.addOption("LEFT_FULL",
    // autonCommands.getAutonomousSequence("LEFT_FULL"));
    // autoChooser.addOption("RIGHT_FULL",
    // autonCommands.getAutonomousSequence("RIGHT_FULL"));
    // autoChooser.addOption("RIGHT_TEST",
    // autonCommands.getAutonomousSequence("RIGHT_TEST"));
    // autoChooser.addOption("LEFT_TEST",
    // autonCommands.getAutonomousSequence("LEFT_TEST"));
    // autoChooser.addOption(
    // "RIGHT_FULL_TEST", autonCommands.getAutonomousSequence("RIGHT_FULL_TEST"));
    // autoChooser.addOption("LEFT_FULL_TEST",
    // autonCommands.getAutonomousSequence("LEFT_FULL_TEST"));
    // autoChooser.addOption("SHUNT_LEFT",
    // autonCommands.getAutonomousSequence("SHUNT_LEFT"));

    autoChooser.addOption("Right Normal", autonCommands.getAutonomousSequence("RIGHT_NO_OUTPOST"));
    autoChooser.addDefaultOption("Right Normal", autonCommands.getAutonomousSequence("RIGHT_CUT"));
    autoChooser.addOption(
        "Right Normal - AMA", autonCommands.getAutonomousSequence("RIGHT_NO_OUTPOST_AMA"));

    autoChooser.addOption("Right Close", autonCommands.getAutonomousSequence("RIGHT_CLOSE"));
    autoChooser.addOption("Right HP", autonCommands.getAutonomousSequence("RIGHT_TEST"));
    autoChooser.addOption("Right No Shunt", autonCommands.getAutonomousSequence("RIGHT_NO_SHUNT"));
    autoChooser.addOption(
        "Right Scavenger", autonCommands.getAutonomousSequence("RIGHT_SCAVENGER"));
    autoChooser.addOption("Left", autonCommands.getAutonomousSequence("LEFT_NEW"));
    autoChooser.addOption("Left Close", autonCommands.getAutonomousSequence("LEFT_NEW_CLOSE"));
    autoChooser.addOption(
        "Middle Human Depot", autonCommands.getAutonomousSequence("DEPOT_HUMAN_MIDDLE"));
    autoChooser.addOption("Left AMA", autonCommands.getAutonomousSequence("LEFT_TEST"));
    autoChooser.addOption("Left Adaptive", autonCommands.getAutonomousSequence("LEFT_ADAPTIVE"));
    autoChooser.addOption("Left Follow", autonCommands.getAutonomousSequence("LEFT_ADAPTIVE2"));
    autoChooser.addOption("Right Follow", autonCommands.getAutonomousSequence("RIGHT_ADAPTIVE2"));
    // autoChooser.addOption("Preload Depot", autonCommands.getAutonomousSequence("DEPOT_PRELOAD"));

    // autoChooser.addOption("Do Auton Stuff",
    // autonCommands.getAutonomousSequence("Do_AUTON_STUFF"));

    // autoChooser.addOption("Test Path",
    // autonCommands.getPathCommand("TuningPath"));
    // autoChooser.addOption("Center Bump Path",
    // autonCommands.getAutonomousSequence("CENTER"));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // DriveCommands.setDriveState(drive, DriveState.DRIVING);
    drive.setDriveState(DriveState.DRIVING);

    Trigger shootTrigger = controller.rightTrigger().or(opController.rightBumper());
    drive.setShootingActiveSupplier(shootTrigger);
    shootTrigger
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (drive.getDriveState() != DriveState.BUMP)
                    drive.setDriveState(DriveState.SHOOTING);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (drive.getDriveState() != DriveState.BUMP)
                    drive.setDriveState(DriveState.DRIVING);
                }));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDriveXLock(
            drive,
            () -> squareInput(-1 * controller.getLeftY()),
            () -> squareInput(-1 * controller.getLeftX()),
            () -> -1 * controller.getRightX()));

    controller
        .leftStick()
        .onTrue(Commands.runOnce(() -> drive.setDriveState(DriveState.ALIGN)))
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -1 * controller.getLeftY(),
                () -> -1 * controller.getLeftX(),
                () -> {
                  // Calculate turret field pose to get direction to goal
                  Pose2d turretFieldPose = shooter.getTurretFieldPose();
                  Translation2d goalPosition =
                      new Translation2d(
                          AllianceFlipUtil.applyX(FieldConstants.Hub.topCenterPoint.getX()),
                          AllianceFlipUtil.applyY(FieldConstants.Hub.topCenterPoint.getY()));

                  // Get angle from turret to goal
                  Rotation2d angleToGoal =
                      goalPosition.minus(turretFieldPose.getTranslation()).getAngle();

                  // Calculate desired robot rotation (account for turret offset and orientation)
                  return angleToGoal
                      .minus(Rotation2d.kCCW_Pi_2) // Account for turret pointing forward
                      .minus(Rotation2d.fromRotations(shooter.getTurretPosition()));
                }))
        .onFalse(Commands.runOnce(() -> drive.setDriveState(DriveState.DRIVING)));

    /*
     * CODE TO ADD ROBOT TURNING TOWARD ITS VELOCITY
     * controller
     * .leftStick()
     * .negate()
     * .and(() -> controller.rightStick().getAsBoolean())
     * .whileTrue(
     * DriveCommands.joystickDriveAtAngle(
     * drive,
     * () -> -1 * controller.getLeftY(),
     * () -> -1 * controller.getLeftX(),
     * () -> {
     * return new Rotation2d(Math.atan2(controller.getLeftX(),
     * controller.getLeftY()));
     * },
     * // drive::getFieldSpeedAngle,
     * 32.0,
     * 100));
     */
    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .rightTrigger()
        .whileTrue(teleopCommands.shootCommand())
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          indexer.setIndexerState(IndexerState.FLUSH);
                        }),
                    teleopCommands.stopAgitateCommand(),
                    Commands.waitTime(Milliseconds.of(500)),
                    teleopCommands.stopShootCommand())
                .onlyWhile(controller.rightTrigger().negate()));

    opController
        .rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  indexer.setIndexerState(IndexerState.JAM);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  if (controller.rightTrigger().getAsBoolean()) {
                    indexer.setIndexerState(IndexerState.INDEXING);
                  } else {
                    indexer.setIndexerState(IndexerState.IDLE);
                  }
                }));

    controller
        .leftTrigger()
        .onTrue(teleopCommands.intakeCommand(IntakeState.INTAKING))
        .onFalse(teleopCommands.intakeCommand(IntakeState.IDLE));

    opController
        .rightBumper()
        .whileTrue(teleopCommands.shootCommand())
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          indexer.setIndexerState(IndexerState.FLUSH);
                        }),
                    teleopCommands.stopAgitateCommand(),
                    Commands.waitTime(Milliseconds.of(500)),
                    teleopCommands.stopShootCommand())
                .onlyWhile(opController.rightTrigger().negate()));

    controller
        .rightBumper()
        .onTrue(teleopCommands.intakeCommand(IntakeState.OUTTAKING))
        .onFalse(teleopCommands.intakeCommand(IntakeState.IDLE));

    opController.leftTrigger().onTrue(teleopCommands.intakeCommand(IntakeState.STOW));

    controller.leftBumper().onTrue(teleopCommands.intakeCommand(IntakeState.STOW));

    opController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.turretBias -= 0.05;
                }));

    opController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.turretBias += 0.05;
                }));

    opController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.hoodBias += 0.05;
                }));

    opController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.hoodBias -= 0.05;
                }));

    opController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.flywheelBias += 5;
                }));
    opController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.flywheelBias -= 5;
                }));

    // opController.leftBumper().onTrue(teleopCommands.climbCommand(ClimbState.TELEOP_CLIMB));
    // opController.rightBumper().onTrue(teleopCommands.climbCommand(ClimbState.STOW));
    // controller
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return drive;
  }

  // public void updateManager() {
  // manager.periodicManager();
  // }

  // public void resetState() {
  // manager.robotState = RobotScoringState.IDLE;
  // manager.intakeState = IntakeManagerState.IDLE;
  // }

  public void resetSubsystems() {
    shooter.setShooterState(ShooterState.STOW);
    intake.setIntakeState(IntakeState.STOW);
    indexer.setIndexerState(IndexerState.IDLE);
    transfer.setTransferState(TransferState.IDLE);
  }

  public static double squareInput(double value) {
    return Math.copySign(value * value, value);
  }

  @AutoLogOutput(key = "Drive/DistanceToHub")
  public double getHubDist() {
    return ShooterTurretCalculator.getDistanceToTarget(
            shooter.getTurretFieldPose(), AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint))
        .in(Meters);
  }

  public void updateDisplays() {
    field.setRobotPose(drive.getPose());
    ArrayList<Pose2d> trajPoses = new ArrayList<>();
    Pose2d zerodDrive =
        new Pose2d(drive.getPose().getX(), drive.getPose().getY(), Rotation2d.kZero);
    for (double i = 0; i <= 8; i++) {
      trajPoses.add(zerodDrive.interpolate(ShooterTurretCalculator.lastLookAhead, i / 8.0));
    }
    field.getObject("Traj").setPoses(trajPoses);
    SmartDashboard.putData("Field/Field", field);

    SmartDashboard.putString("Field/Data", DriverStation.getGameSpecificMessage());
    SmartDashboard.putBoolean("Field/IsActive", isHubActive());
  }

  public boolean isHubActive() {
    double matchTime = DriverStation.getMatchTime();
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }

    if (DriverStation.isDisabled()) {
      putPhaseTimeLeft(0);
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      putPhaseTimeLeft(matchTime);
      return true;
    }
    // // At this point, if we're not teleop enabled, there is no hub.
    // if (!DriverStation.isTeleopEnabled()) {
    // return false;
    // }

    // We're teleop enabled, compute.
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in
    // teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      putPhaseTimeLeft(matchTime - (130 - (shift1Active ? 25 : 0)));
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      putPhaseTimeLeft(matchTime - 105);
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      putPhaseTimeLeft(matchTime - 80);
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      putPhaseTimeLeft(matchTime - 55);
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      putPhaseTimeLeft(matchTime - 30);
      // Shift 4
      return !shift1Active;
    } else {
      putPhaseTimeLeft(matchTime);
      // End game, hub always active.
      return true;
    }
  }

  public void putPhaseTimeLeft(double timeLeft) {
    SmartDashboard.putNumber("Field/PhaseTimeLeft", timeLeft);
  }

  // @AutoLogOutput(key = "TotalCurrent")
  // public double getTotalCurrent() {
  // pdh.clearStickyFaults();
  // return pdh.getTotalCurrent();
  // }
}

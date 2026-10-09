package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChoreoFiles.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.Transfer.TransferState;
import frc.robot.util.geometry.AllianceFlipUtil;
import java.util.Optional;

public class AutonCommands extends TeleopCommands {

  private Drive drive;
  private Shooter shooter;
  private Intake intake;
  private Transfer transfer;
  private Timer autonTimer;

  public AutonCommands(
      Drive drive, Intake intake, Indexer indexer, Transfer transfer, Shooter shooter) {
    super(drive, intake, indexer, transfer, shooter, () -> false);
    this.drive = drive;
    this.shooter = shooter;
    this.intake = intake;
    this.transfer = transfer;
    autonTimer = new Timer();
  }

  public Command getPathCommand(String pathName) {

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Command getPathCommand(String pathName, int index) {
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName, index);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public Optional<PathPlannerPath> getTraj(String pathName) {
    try {
      return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
    } catch (Exception e) {
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public Command getAutonomousSequence(String startChoice) {
    SequentialCommandGroup autonCommand = new SequentialCommandGroup();

    switch (startChoice) {
      case "CENTER":
        // autonCommand.addCommands(getPathCommand("C_Start_Climb"));
        break;

      case "RIGHT_FULL":
        // autonCommand.addCommands((ChoreoTraj.H_Intake_4590_H_BUMP$0));

        autonCommand.addCommands(getPathCommand("H_Start_H_BUMP"));
        autonCommand.addCommands(getPathCommand("H_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("H_BUMP_H_Intake_45"));
        autonCommand.addCommands(getPathCommand("H_Intake_45_D_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("D_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;
      case "RIGHT_45":
        autonCommand.addCommands(getPathCommand("H_Start_H_BUMP"));
        autonCommand.addCommands(getPathCommand("H_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("H_BUMP_H_Intake_45"));
        autonCommand.addCommands(getPathCommand("H_Intake_45_H_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("H_BumpNeutralAlliance"));
        autonCommand.addCommands(stopDrive());
        autonCommand.addCommands(shootCommand());
        break;
      case "LEFT_45":
        autonCommand.addCommands(getPathCommand("D_Start_D_BUMP"));
        autonCommand.addCommands(getPathCommand("D_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("D_BUMP_D_Intake_45"));
        autonCommand.addCommands(getPathCommand("D_Intake_45_D_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("D_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;
      case "LEFT_FULL":
        autonCommand.addCommands(getPathCommand("D_Start_D_BUMP"));
        autonCommand.addCommands(getPathCommand("D_BumpAllianceNeutral"));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand("D_BUMP_D_Intake_45"));
        autonCommand.addCommands(getPathCommand("D_Intake_45_H_BUMP"));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand("H_BumpNeutralAlliance"));
        autonCommand.addCommands(shootCommand());
        break;

      case "RIGHT_TEST":
        autonCommand.addCommands(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      AllianceFlipUtil.apply(ChoreoTraj.H_Partial_1Pass.initialPoseBlue()));
                }));
        autonCommand.addCommands(humanPlayerAuton("H_Partial_1Pass"));
        break;
      case "RIGHT_NO_OUTPOST":
        autonCommand.addCommands(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass.initialPoseBlue()));
                }));
        // autonCommand.addCommands(getAutonCommandSegments("H_Partial_2Pass"));
        autonCommand.addCommands(getHumanSide());

        break;
      case "RIGHT_NO_OUTPOST_FLIPPED":
        autonCommand.addCommands(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass_flipped.initialPoseBlue()));
                }));
        // autonCommand.addCommands(getAutonCommandSegments("H_Partial_2Pass"));
        autonCommand.addCommands(getHumanSideFlipped());

        break;
      case "RIGHT_NO_OUTPOST_AMA":
        autonCommand.addCommands(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2PassAMA.initialPoseBlue()));
                }));
        autonCommand.addCommands(getHumanSideAMA());
        break;
      case "RIGHT_FULL_TEST":
        autonCommand.addCommands(getAutonCommandSegments("H_Full_1Pass"));
        break;
      case "LEFT_FULL_TEST":
        autonCommand.addCommands(getAutonCommandSegments("D_Full_1Pass"));
        break;
      case "LEFT_TEST":
        autonCommand.addCommands(
            Commands.runOnce(
                () -> {
                  drive.setPose(
                      AllianceFlipUtil.apply(ChoreoTraj.D_Partial_1Pass.initialPoseBlue()));
                }));
        // autonCommand.addCommands(getAutonCommandSegments("D_Partial_1Pass"));
        autonCommand.addCommands(getDepotSide());
        break;
      case "SHUNT_LEFT":
        String quick = "H_Shunt_Grab";
        autonCommand.addCommands(getPathCommand(quick, 0));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand(quick, 1));
        autonCommand.addCommands(intakeCommand(IntakeState.OUTTAKING));
        autonCommand.addCommands(getPathCommand(quick, 2));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(getPathCommand(quick, 3));
        autonCommand.addCommands(intakeCommand(IntakeState.STOW));
        autonCommand.addCommands(getPathCommand(quick, 4));
        autonCommand.addCommands(intakeCommand(IntakeState.INTAKING));
        autonCommand.addCommands(shootCommand());
        autonCommand.addCommands(getPathCommand(quick, 5));
        break;
      case "RIGHT_SCAVENGER":
        autonCommand.addCommands(humanCenterScavenger());
        break;
      case "Do_AUTON_STUFF":
        autonCommand.addCommands(getAutonStuff());
        break;
      case "RIGHT_CLOSE":
        autonCommand.addCommands(getHumanSideClose());
        break;
      case "RIGHT_CUT":
        autonCommand.addCommands(getHumanSideCut());
        break;
      case "LEFT_NEW":
        autonCommand.addCommands(getDepotSideNew());
        break;
      case "LEFT_NEW_CLOSE":
        autonCommand.addCommands(getDepotSideNewClose());
        break;
      case "LEFT_ADAPTIVE":
        autonCommand.addCommands(getDepotSideAdaptive(Time.ofBaseUnits(4, Second)));
        break;
      case "DEPOT_HUMAN_MIDDLE":
        autonCommand.addCommands(depotHuman());
        break;
      case "LEFT_ADAPTIVE2":
        autonCommand.addCommands(
            getDepotSideAdaptive2(Time.ofBaseUnits(1.5, Second), Time.ofBaseUnits(4, Second)));
        break;
      case "RIGHT_ADAPTIVE2":
        autonCommand.addCommands(getHumanSideAdaptive2(Time.ofBaseUnits(6, Second)));
        break;
      case "DEPOT_PRELOAD":
        autonCommand.addCommands(getDepotPreload());
        break;
      default:
        DriverStation.reportError("Big oops: Invalid Start Pos", false);
        break;
    }

    return autonCommand;
  }

  public static Pose2d swapToCorrectPose(Pose2d pose, Alliance curalliance) {
    if (curalliance == DriverStation.getAlliance().get()) {
      return pose;
    } else {
      return FlippingUtil.flipFieldPose(pose);
    }
  }

  // public Command setInitialPose(ChoreoTraj traj)
  // {
  //   return Commands.runOnce(
  //                           () -> {
  //
  // drive.setPose(swapToCorrectPose(ChoreoTraj.D_Start_Intake45.initialPoseBlue(),
  // Alliance.Blue));
  //                           });
  // }

  public Command depotHuman() {
    String quick = "Depot_Human_Middle";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.Depot_Human_Middle.initialPoseBlue()));
            }));

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(shootCommand());
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(2));
    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));
    command.addCommands(intakeCommand(IntakeState.AGITATE));

    command.addCommands(stopDrive());
    return command;
  }

  public Command getAutonCommandSegments(String overallName) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(getPathCommand(overallName, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(overallName, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(overallName, 2));
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.stop();
            }));
    command.addCommands(stopShootCommand());
    // command.addCommands(new WaitCommand(1.5));
    command.addCommands(indexCommand(IndexerState.INDEXING));
    command.addCommands(shootCommand());

    // command.addCommands(getPathCommand(overallName, 3));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));

    // command.addCommands(Commands.waitSeconds(5));
    // command.addCommands(intakeCommand(IntakeState.STOW));
    // command.addCommands(stopShootCommand());

    return command;
  }

  public Command getAutonStuff() {
    String quick = "AutonStuff";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.AutonStuff.initialPoseBlue()));
            }));

    command.addCommands(
        Commands.runOnce(
            () -> {
              autonTimer.restart();
            }));

    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    command.addCommands(new WaitCommand(0.5));
    command.addCommands(getPathCommand(quick, 2));

    Command com =
        Commands.parallel(
            getPathCommand(quick, 3), indexCommand(IndexerState.INDEXING), shootCommand());
    command.addCommands(com);

    command.addCommands(stopDrive());
    // ommand.addCommands(
    // Commands.waitUntil(
    //     () -> {
    //       return autonTimer.hasElapsed(14);
    //     }));
    command.addCommands(new WaitCommand(8));
    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 4));
    return command;
  }

  public Command getDepotSide() {
    String quick = "D_Partial_1Pass";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.D_Partial_1Pass.initialPoseBlue()));
            }));
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));

    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    command.addCommands(stopShootCommand());

    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootCommand());
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootCommand());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSide() {
    String quick = "H_Partial_2Pass";

    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass_R.initialPoseBlue()));
            }));

    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideFlipped() {
    String quick = "D_Partial_2Pass_flipped";

    SequentialCommandGroup command = new SequentialCommandGroup();

    // command.addCommands(
    //     Commands.runOnce(
    //         () -> {
    //
    // drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass_R.initialPoseBlue()));
    //         }));

    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideClose() {
    String quick = "H_Partial_2Pass_Close2";

    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass_Close2.initialPoseBlue()));
            }));

    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideAMA() {
    String quick = "H_Partial_2PassAMA";

    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2PassAMA.initialPoseBlue()));
            }));
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    // if (Robot.isSimulation()) {
    //   command.addCommands(
    //       Commands.runOnce(
    //           () -> {
    //             Pose2d current = drive.getPose()
    //                 // .plus(new Transform2d(0.5, 0.5, Rotation2d.kZero))
    //                 ;
    //             double randomAngle = Math.random() * 2 * Math.PI;
    //             drive.setPose(new Pose2d(current.getTranslation(), new Rotation2d(randomAngle)));
    //           }));
    // }
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    command.addCommands(stopShootCommand());

    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(2.5));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(stopDrive());
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getDepotSideNew() {
    String quick = "D_Partial_2Pass";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideCut() {
    String quick = "H_Partial_2Pass_Cut";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass_Cut.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getDepotSideNewClose() {
    String quick = "D_Partial_2Pass_Close2";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass_Close2.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 4));
    command.addCommands(shootIndex());
    command.addCommands(stopDrive());
    command.addCommands(Commands.waitSeconds(1.5));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideAccel() {
    String quick = "H_Partial_2Pass";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass.initialPoseBlue()));
            }));
    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(quick, 2));
    // if (RobotBase.isSimulation()) {
    //   command.addCommands(
    //       Commands.runOnce(
    //           () -> {
    //             Pose2d current = drive.getPose()
    //                 // .plus(new Transform2d(0.5, 0.5, Rotation2d.kZero))
    //                 ;
    //             double randomAngle = Math.random() * 2 * Math.PI;
    //             drive.setPose(new Pose2d(current.getTranslation(), new Rotation2d(randomAngle)));
    //           }));
    // }
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    command.addCommands(stopShootCommand());

    SequentialCommandGroup restOfCommand = new SequentialCommandGroup();

    restOfCommand.addCommands(new WaitCommand(2.5));
    restOfCommand.addCommands(intakeCommand(IntakeState.INTAKING));
    restOfCommand.addCommands(getPathCommand(quick, 4));
    restOfCommand.addCommands(stopDrive());
    restOfCommand.addCommands(intakeCommand(IntakeState.AGITATE));

    // command.addCommands(Commands.parallel(restOfCommand, runShootCheckCommand()));

    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getDepotSideAdaptive(Time timeDelay) {
    String quick = "D_Partial_2Pass_copy1";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass_copy1.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());

    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(timeDelay));
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    // command.addCommands(getPathCommand(quick, 1));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(3.5));
    command.addCommands(shootIndex());

    // command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(getPathCommand(quick, 4));
    // command.addCommands(shootIndex());
    // command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(1.5));
    // command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getDepotSideAdaptive2(Time timeDelay, Time timeDelay2) {
    String quick = "D_Partial_2Pass_Adaptive2";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass_Adaptive2.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());

    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(timeDelay));
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    // command.addCommands(getPathCommand(quick, 1));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(timeDelay2));
    command.addCommands(intakeCommand(IntakeState.IDLE));
    command.addCommands(getPathCommand(quick, 3));
    command.addCommands(getPathCommand(quick, 4));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));

    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(2));
    command.addCommands(getPathCommand(quick, 5));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(getPathCommand(quick, 4));
    // command.addCommands(shootIndex());
    // command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(1.5));
    // command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  // public Command getDepotSideAdaptivePickup2(Time timeDelay, Time timeDelay2) {
  //   String quick = "D_Partial_2Pass_Adaptive2Pickup";

  //   SequentialCommandGroup command = new SequentialCommandGroup();
  //   command.addCommands(
  //       Commands.runOnce(
  //           () -> {
  //             drive.setPose(
  //
  // AllianceFlipUtil.apply(ChoreoTraj.D_Partial_2Pass_Adaptive2.initialPoseBlue()));
  //           }));
  //   command.addCommands(stopShootCommand());

  //   command.addCommands(getPathCommand(quick, 0));
  //   command.addCommands(stopDrive());
  //   command.addCommands(new WaitCommand(timeDelay));
  //   command.addCommands(getPathCommand(quick, 1));
  //   command.addCommands(intakeCommand(IntakeState.INTAKING));
  //   // command.addCommands(shootIndex());

  //   // command.addCommands(getPathCommand(quick, 1));

  //   command.addCommands(getPathCommand(quick, 2));
  //   command.addCommands(stopDrive());
  //   command.addCommands(new WaitCommand(timeDelay2));
  //   command.addCommands(intakeCommand(IntakeState.IDLE));
  //   command.addCommands(getPathCommand(quick, 3));
  //   command.addCommands(getPathCommand(quick, 4));

  //   command.addCommands(stopDrive());
  //   // command.addCommands(Commands.waitSeconds(0.5));

  //   Command com = Commands.parallel(indexCommand(IndexerState.INDEXING),
  // shootAndOuttakeCommand());
  //   command.addCommands(com);
  //   command.addCommands(new WaitCommand(2));
  //   command.addCommands(getPathCommand(quick, 5));
  //   // command.addCommands(intakeCommand(IntakeState.INTAKING));
  //   // command.addCommands(getPathCommand(quick, 4));
  //   // command.addCommands(shootIndex());
  //   // command.addCommands(stopDrive());
  //   // command.addCommands(Commands.waitSeconds(1.5));
  //   // command.addCommands(intakeCommand(IntakeState.AGITATE));
  //   // command.addCommands(new WaitCommand(1.5));
  //   return command;
  // }

  public Command getDepotPreload() {
    String quick = "D_PreloadDepot";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.D_PreloadDepot.initialPoseBlue()));
            }));
    command.addCommands(shootCommand());

    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.AGITATE));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(2));
    // command.addCommands(shootIndex());

    // command.addCommands(getPathCommand(quick, 1));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(getPathCommand(quick, 4));
    // command.addCommands(shootIndex());
    // command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(1.5));
    // command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command getHumanSideAdaptive2(Time timeDelay) {
    String quick = "H_Partial_2Pass_Adaptive2";

    SequentialCommandGroup command = new SequentialCommandGroup();
    command.addCommands(
        Commands.runOnce(
            () -> {
              drive.setPose(
                  AllianceFlipUtil.apply(ChoreoTraj.H_Partial_2Pass_Adaptive2.initialPoseBlue()));
            }));
    command.addCommands(stopShootCommand());

    command.addCommands(getPathCommand(quick, 0));
    command.addCommands(stopDrive());
    command.addCommands(new WaitCommand(timeDelay));
    command.addCommands(getPathCommand(quick, 1));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(shootIndex());

    // command.addCommands(getPathCommand(quick, 1));

    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(intakeCommand(IntakeState.IDLE));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(0.5));

    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);
    command.addCommands(new WaitCommand(2));
    command.addCommands(getPathCommand(quick, 4));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(getPathCommand(quick, 4));
    // command.addCommands(shootIndex());
    // command.addCommands(stopDrive());
    // command.addCommands(Commands.waitSeconds(1.5));
    // command.addCommands(intakeCommand(IntakeState.AGITATE));
    // command.addCommands(new WaitCommand(1.5));
    return command;
  }

  public Command testAuton() {
    String quick = "FirstTestAuton";

    // A "SequentialCommandGroup" is a command that runs each command in the sequence one by one,
    // waiting until
    // each command is finished before moving to the next
    SequentialCommandGroup command = new SequentialCommandGroup();

    // The first "segment" of the FirstAuton path. The quick string is used so I don't have to write
    // out a longer name again and again
    // Just like arrays, these segments start off with index 0 and increase by 1
    command.addCommands(getPathCommand(quick, 0));

    // Adding a stop drive command between segments you want a stop between helps ensure the drive
    // doesn't drift
    command.addCommands(stopDrive());

    // Use wait commands for delays between segments
    command.addCommands(new WaitCommand(0.5));

    // Do the second segment of the path
    command.addCommands(getPathCommand(quick, 1));

    // Intaking command between segments. This command runs asynchrononously, meaning there is no
    // real delay between segment 1 and 2
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    // Run the next segment, stop intaking, and seamlessly continue to segment 3
    command.addCommands(getPathCommand(quick, 2));
    command.addCommands(intakeCommand(IntakeState.IDLE));
    command.addCommands(getPathCommand(quick, 3));

    command.addCommands(stopDrive());

    // Making smaller parallel commands can help to ensure commands happen at the exact same time.
    Command com = Commands.parallel(indexCommand(IndexerState.INDEXING), shootAndOuttakeCommand());
    command.addCommands(com);

    // Wait and final segment
    command.addCommands(new WaitCommand(2));
    command.addCommands(getPathCommand(quick, 4));

    // Return the whole sequential command group
    return command;
  }

  public Command humanPlayerAuton(String name) {
    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(getPathCommand(name, 0));
    command.addCommands(intakeCommand(IntakeState.INTAKING));

    command.addCommands(getPathCommand(name, 1));
    command.addCommands(intakeCommand(IntakeState.IDLE));

    command.addCommands(getPathCommand(name, 2));

    command.addCommands(stopShootCommand());
    // command.addCommands(new WaitCommand(1.5));
    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(
        Commands.parallel(
            indexCommand(IndexerState.INDEXING), shootCommand(), getPathCommand(name, 3)));

    // command.addCommands(
    //     Commands.runOnce(
    //         () -> {
    //           drive.stop();
    //         }));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));
    // command.addCommands(
    // Commands.parallel(
    //     new WaitCommand(0.5)
    //         .andThen(indexCommand(IndexerState.INDEXING).andThen(shootCommand())),
    //     stopShootCommand(),
    //     getPathCommand(name, 3)
    //         .andThen(

    //             Commands.runOnce(
    //                 () -> {
    //                   drive.stop();
    //                 }))));
    // command.addCommands(getPathCommand(name, 3));
    // command.addCommands(
    //     Commands.runOnce(
    //         () -> {
    //           drive.stop();
    //         }));
    // command.addCommands(getPathCommand(overallName, 3));
    // command.addCommands(intakeCommand(IntakeState.INTAKING));

    // command.addCommands(Commands.waitSeconds(5));
    // command.addCommands(intakeCommand(IntakeState.STOW));
    // command.addCommands(stopShootCommand());

    return command;
  }

  public Command humanCenterScavenger() {
    String c = "KrishIdea";

    SequentialCommandGroup command = new SequentialCommandGroup();

    command.addCommands(
        Commands.runOnce(
            () -> {
              autonTimer.restart();
            }));
    command.addCommands(
        Commands.runOnce(
            () -> drive.setPose(AllianceFlipUtil.apply(ChoreoTraj.KrishIdea.initialPoseBlue()))));
    command.addCommands(getPathCommand(c, 0));
    command.addCommands(stopDrive());
    command.addCommands(
        Commands.waitUntil(
            () -> {
              return autonTimer.hasElapsed(4);
            }));

    command.addCommands(intakeCommand(IntakeState.INTAKING));
    command.addCommands(getPathCommand(c, 1));
    command.addCommands(stopDrive());
    command.addCommands(shootCommand());

    return command;
  }

  public Command stopDrive() {
    return Commands.runOnce(
        () -> {
          drive.stop();
        });
  }

  @Override
  public Command shootCommand() {
    return Commands.parallel(
        shooter.startShooterOnce(),
        Commands.runOnce(
            () -> {
              if (!intakeStateSupplier.get().equals(IntakeState.INTAKING)
                  && !intakeStateSupplier.get().equals(IntakeState.OUTTAKING))
                intake.setIntakeState(IntakeState.AGITATE);
              transfer.setTransferState(TransferState.TRANSFERRING);
            }));
  }

  public Command shootAndOuttakeCommand() {
    return Commands.parallel(
        shooter.startShooterOnce(),
        Commands.runOnce(
            () -> {
              if (!intakeStateSupplier.get().equals(IntakeState.INTAKING))
                intake.setIntakeState(IntakeState.OUTTAKING);
              transfer.setTransferState(TransferState.TRANSFERRING);
            }));
  }

  public Command intakeCommand(IntakeState state, double seconds) {
    IntakeState original = intake.getIntakeState();
    return intakeCommand(state).withTimeout(seconds).andThen(intakeCommand(original));
  }

  public Command shootIndex() {
    return Commands.parallel(shootCommand(), indexCommand(IndexerState.INDEXING));
  }
  // public Command runShootCheckCommand() {
  //   HashMap<Integer, Command> commandMap = new HashMap<>();
  //   commandMap.put(1, shootCommand());
  //   commandMap.put(
  //       2,
  //       Commands.runOnce(
  //               () -> {
  //                 indexer.setIndexerState(IndexerState.FLUSH);
  //               })
  //           .andThen(stopAgitateCommand())
  //           .andThen(Commands.waitTime(Milliseconds.of(100)))
  //           .andThen(stopShootCommand()));

  //   BooleanSupplier accelCheck =
  //       () -> {
  //         return (drive.getAccelComponents().getTranslation().getNorm() > 3);
  //       };
  //   return Commands.repeatingSequence(
  //       shootCommand(),
  //       Commands.waitUntil(accelCheck),
  //       commandMap.get(1),
  //       Commands.waitUntil(
  //           () -> {
  //             return !accelCheck.getAsBoolean();
  //           }));
  // }
}

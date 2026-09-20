// package frc.robot;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.constants.FieldConstants;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.Drive.DriveState;
// import frc.robot.subsystems.drive.Drive.Zone;
// import frc.robot.subsystems.indexer.Indexer;
// import frc.robot.subsystems.indexer.Indexer.IndexerState;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.intake.Intake.IntakeState;
// import frc.robot.subsystems.shooter.Shooter;
// import frc.robot.subsystems.shooter.Shooter.ShooterState;
// import frc.robot.subsystems.transfer.Transfer;
// import frc.robot.subsystems.transfer.Transfer.TransferState;
// import frc.robot.util.geometry.AllianceFlipUtil;
// import org.littletonrobotics.junction.AutoLogOutput;

// public class RobotManager {

//   private Drive drive;
//   private Intake intake;
//   private Indexer indexer;
//   private Transfer transfer;
//   private Shooter shooter;

//   public RobotScoringState robotState;
//   private ShooterState shooterState;
//   public IntakeManagerState intakeState;

//   public RobotManager(
//       Drive drive, Intake intake, Indexer indexer, Transfer transfer, Shooter shooter) {
//     this.drive = drive;
//     this.intake = intake;
//     this.indexer = indexer;
//     this.transfer = transfer;
//     this.shooter = shooter;
//     robotState = RobotScoringState.IDLE;
//     intakeState = IntakeManagerState.IDLE;
//   }

//   public void periodicManager() {
//     // MARK: - INTAKE
//     if (robotState == RobotScoringState.CLIMBING) {
//       intakeState = IntakeManagerState.IDLE;
//     }

//     if (intakeState != IntakeManagerState.INTAKING) {
//       switch (robotState) {
//         case IDLE:
//         case CLIMBING:
//           intake.setIntakeState(IntakeState.STOW);
//           break;
//         case SHOOTING:
//         case FIXED_SHOOTING:
//           intake.setIntakeState(IntakeState.AGITATE);
//           break;
//         default:
//           break;
//       }
//     } else {
//       intake.setIntakeState(IntakeState.INTAKING);
//     }

//     // MARK: - SPINDEXER
//     switch (robotState) {
//       case CLIMBING:
//       case IDLE:
//         indexer.setIndexerState(IndexerState.IDLE);
//         break;
//       case FIXED_SHOOTING:
//       case SHOOTING:
//         indexer.setIndexerState(IndexerState.INDEXING);
//         break;
//       default:
//         break;
//     }

//     // MARK: - TRANSFER
//     switch (robotState) {
//       case CLIMBING:
//       case IDLE:
//         transfer.setTransferState(TransferState.IDLE);
//         break;
//       case FIXED_SHOOTING:
//       case SHOOTING:
//         transfer.setTransferState(TransferState.TRANSFERRING);
//         break;
//       default:
//         break;
//     }

//     // MARK: - SHOOTER
//     switch (robotState) {
//       case CLIMBING:
//         // shooterState = ShooterState.IDLE;
//         // break;
//       case IDLE:
//         shooterState = ShooterState.IDLE_HUB;
//         break;
//       case FIXED_SHOOTING:
//         shooterState = ShooterState.SHOOT_FIXED;
//         break;
//         // The following two states need more complex logic
//       case SHOOTING:
//         Zone zone = drive.returnZone(drive.getPose());
//         switch (zone) {
//           case ALLIANCE:
//             shooterState = ShooterState.SHOOT_HUB;
//             break;
//           case NEUTRAL_LEFT:
//             shooterState = ShooterState.SHOOT_PASS_L;
//             break;
//           case NEUTRAL_RIGHT:
//             shooterState = ShooterState.SHOOT_PASS_R;
//             break;
//           default:
//             break;
//         }
//         break;
//       default:
//         break;
//     }
//     shooter.setShooterState(shooterState);

//     // MARK: - DRIVE
//     switch (robotState) {
//       case FIXED_SHOOTING:
//       case CLIMBING:
//         drive.setDriveState(DriveState.ALIGN);
//       case IDLE:
//         drive.setDriveState(DriveState.DRIVING);
//         break;
//       case SHOOTING:
//         drive.setDriveState(DriveState.SHOOTING);
//         break;
//       default:
//         drive.setDriveState(DriveState.IDLE);
//         break;
//     }

//     // MARK: - CLIMB()
//     // ADD CLIMBER HERE WHEN DONE
//   }

//   public enum RobotScoringState {
//     IDLE,
//     SHOOTING,
//     CLIMBING,
//     FIXED_SHOOTING,
//   }

//   public enum IntakeManagerState {
//     IDLE,
//     INTAKING
//   }

//   public enum ShootGoal {
//     HUB(AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)),
//     LEFT_PASS(AllianceFlipUtil.apply(FieldConstants.LeftBump.farLeftCorner)),
//     RIGHT_PASS(AllianceFlipUtil.apply(FieldConstants.RightBump.farRightCorner));

//     private Translation3d goal;

//     private ShootGoal(Translation3d goal) {
//       this.goal = goal;
//     }

//     private ShootGoal(Translation2d goal) {
//       this.goal = new Translation3d(goal);
//     }

//     public Translation3d getGoal() {
//       return goal;
//     }
//   }

//   public Runnable toState(RobotScoringState state) {
//     return () -> {
//       robotState = state;
//     };
//   }

//   public Command toStateCommand(RobotScoringState state) {
//     return Commands.runOnce(toState(state));
//   }

//   public Command setIntakeCommand(IntakeManagerState state) {
//     return Commands.runOnce(
//         () -> {
//           intakeState = state;
//         });
//   }

//   @AutoLogOutput(key = "ROBOTSTATE")
//   public RobotScoringState getState() {
//     return robotState;
//   }

//   @AutoLogOutput(key = "INTAKESTATE")
//   public IntakeManagerState getIntakeState() {
//     return intakeState;
//   }
// }

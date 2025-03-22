
package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.auto.AutoBuildingBlocks;
import frc.robot.commands.SuckCmd;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.CenterAuto;
import frc.robot.commands.auto.Drive;
import frc.robot.commands.auto.Left3Piece;
import frc.robot.commands.auto.Right3Piece;
import frc.robot.commands.auto.Testing;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstantsComp;
import frc.robot.drive.TunerConstantsPrac;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.DPadButton;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.DPadButton.DPad;
import frc.robot.input.Keybind;
import frc.robot.input.SnapButton;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import frc.robot.subsystems.PigeonSubsystem;

public class RobotContainer {
  public final CommandXboxController driverController = new CommandXboxController(
      Constants.DRIVER_CONTROLLER_PORT);
  public final CommandXboxController codriverController = new CommandXboxController(
      Constants.CODRIVER_CONTROLLER_PORT);

  public SnapButton snap = SnapButton.None;

  double speedFactor = 1.0;

  private final double MaxSpeed;

  private final double MaxAngularRate;

  private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()

      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()

      .withDriveRequestType(DriveRequestType.Velocity);

  private final Telemetry logger;

  public final CommandSwerveDrivetrain drivetrain;
  double heightFactor;

  final SendableChooser<AutoCommand> autoChooser;

  public double positionError = Double.MAX_VALUE;

  public RobotContainer() {

    if (Robot.isCompBot) {
      drivetrain = TunerConstantsComp.createDrivetrain();
      MaxSpeed = TunerConstantsComp.kSpeedAt12Volts.in(MetersPerSecond) * speedFactor;
    } else {
      drivetrain = TunerConstantsPrac.createDrivetrain();
      MaxSpeed = TunerConstantsPrac.kSpeedAt12Volts.in(MetersPerSecond) * speedFactor;
    }
    MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * speedFactor;
    logger = new Telemetry(MaxSpeed);
    AutoBuildingBlocks.drivetrain = drivetrain;

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);

    autoChooser.addOption("Drive", new Drive(drivetrain, driveRC));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    updateDrivetrainRobotPerspective();
  }

  public void updateDrivetrainRobotPerspective() {
    Rotation2d forward;
    if (Robot.alliance == Alliance.Red) {
      forward = new Rotation2d(Math.PI);
    } else {
      forward = new Rotation2d(0);
    }
    drivetrain.setOperatorPerspectiveForward(forward);
  }

  private void configureBindings() {

    driverController.start().and(driverController.back().negate()).onTrue(new InstantCommand(() -> {
      updateDrivetrainRobotPerspective();
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
      if (llMeasurement != null) {
        drivetrain.resetPose(llMeasurement.pose);
      }
    }));

    drivetrain.setDefaultCommand(

        drivetrain.applyRequest(() -> {

          double rotate = ExtraMath.deadzone(
              -driverController.getRightX() * heightFactor * MaxAngularRate,
              0.1);
          double horizontal = ExtraMath.deadzone(
              -driverController.getLeftX() * heightFactor * MaxSpeed, 0.1);
          double vertical = ExtraMath.deadzone(
              -driverController.getLeftY() * heightFactor * MaxSpeed, 0.1);

          return driveFC.withVelocityX(vertical)
              .withVelocityY(horizontal)
              .withRotationalRate(rotate);

        }));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

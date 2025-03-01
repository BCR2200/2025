// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.commands.auto.AutoBuildingBlocks;
import frc.robot.commands.SuckCmd;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.auto.CenterAuto;
import frc.robot.commands.auto.L3LAuto;
import frc.robot.commands.auto.LeftAuto;
import frc.robot.commands.auto.RightAuto;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstants;
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
import frc.robot.subsystems.ReaLEDSubsystem;

public class RobotContainer {
  public final CommandXboxController driverController = new CommandXboxController(
      Constants.DRIVER_CONTROLLER_PORT);
  public final CommandXboxController codriverController = new CommandXboxController(
      Constants.CODRIVER_CONTROLLER_PORT);
  // public final CommandXboxController testController = new
  // CommandXboxController(Constants.TEST_CONTROLLER_PORT);

  // set up Subsystems
  public PigeonSubsystem gyro;
  public ClimberSubsystem climber;
  public ElevClArmSubsystem e;
  public ReaLEDSubsystem led;
  public PowerDistribution pdp;

  // modes keybind
  Keybind selectButton;
  Keybind startButton;

  // buttons
  Keybind aButton;
  Keybind bButton;
  Keybind xButton;
  Keybind yButton;

  Keybind snapA;
  Keybind snapB;
  Keybind snapX;
  Keybind snapY;
  
  //codriver stick click
  Keybind rightStickClick;
  Keybind leftStickClick;

  DPadButton rightDpad;
  DPadButton leftDpad;
  DPadButton upDpad;
  DPadButton downDpad;

  DPadButton climb;
  DPadButton unclimb;

  double dpadShiftX;
  double dpadShiftY;

  // shoot keybind
  AnalogTrigger rightTrigger;
  AnalogTrigger leftTrigger;
  AnalogTrigger feederRightTrigger;
  AnalogTrigger feederLeftTrigger;

  Keybind rightBumper;
  Keybind leftBumper;

  SnapButton snap = SnapButton.None;

  double speedFactor = 1.0;

  

  static Rotation2d LeftFeederAngle = Rotation2d.fromDegrees(90 - 144.011);
  static Rotation2d RightFeederAngle = Rotation2d.fromDegrees(144.011 - 90); // measurements stolen from spectrum
  static Rotation2d ReefFAngle = Rotation2d.fromDegrees(0);
  static Rotation2d ReefFRAngle = Rotation2d.fromDegrees(60);
  static Rotation2d ReefFLAngle = Rotation2d.fromDegrees(-60);
  static Rotation2d ReefBLAngle = Rotation2d.fromDegrees(-120);
  static Rotation2d ReefBRAngle = Rotation2d.fromDegrees(120);
  static Rotation2d ReefBAngle = Rotation2d.fromDegrees(180);
  Rotation2d direction;

  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * speedFactor; // kSpeedAt12Volts
                                                                                                    // desired top
                                                                                                    // speed
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * speedFactor; // 3/4 of
                                                                                                        // a
                                                                                                        // rotation
                                                                                                        // per
                                                                                                        // second
  // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  private final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
      // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
      .withDriveRequestType(DriveRequestType.Velocity); // Use Closed loop control for drive
                                                               // motors

  private final SwerveRequest.FieldCentricFacingAngle driveFCFA = new SwerveRequest.FieldCentricFacingAngle()
      // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                               // motors

  // private final SwerveRequest.SwerveDriveBrake brake = new
  // SwerveRequest.SwerveDriveBrake();
  public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  double heightFactor;

  final SendableChooser<AutoCommand> autoChooser;

  public Timer backItUpTimer;

  public RobotContainer() {
    gyro = new PigeonSubsystem();
    pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);
    e = new ElevClArmSubsystem();
    climber = new ClimberSubsystem();
    led = new ReaLEDSubsystem();

    backItUpTimer = new Timer();
    backItUpTimer.start();

    AutoBuildingBlocks.drivetrain = drivetrain;

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("LeftAuto", new LeftAuto(e, drivetrain, driveRC));
    autoChooser.addOption("RightAuto", new RightAuto(e, drivetrain, driveRC));
    autoChooser.addOption("CenterAuto", new CenterAuto(e, drivetrain, driveRC));
    autoChooser.addOption("L3LAuto", new L3LAuto(e, drivetrain, driveRC));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  double idToLookFor;

  private void configureBindings() {
    // climber engage - toggle? change controls when climbed? make sure arm is
    // forward, and everything is safe
    selectButton = new Keybind(codriverController, Button.Select);
    startButton = new Keybind(codriverController, Button.Start);
    aButton = new Keybind(codriverController, Button.A);
    bButton = new Keybind(codriverController, Button.B);
    xButton = new Keybind(codriverController, Button.X);
    yButton = new Keybind(codriverController, Button.Y);

    snapA = new Keybind(driverController, Button.A);
    snapB = new Keybind(driverController, Button.B);
    snapX = new Keybind(driverController, Button.X);
    snapY = new Keybind(driverController, Button.Y);

    leftStickClick = new Keybind(codriverController, Button.LeftStick);
    rightStickClick = new Keybind(codriverController, Button.RightStick);

    leftDpad = new DPadButton(driverController, DPad.Left);
    rightDpad = new DPadButton(driverController, DPad.Right);
    upDpad = new DPadButton(driverController, DPad.Up);
    downDpad = new DPadButton(driverController, DPad.Down);

    climb = new DPadButton(codriverController, DPad.Up);
    unclimb = new DPadButton(codriverController, DPad.Down);

    // processor (just shoot in safe?) maybe default to processor rather than
    // algaesafe
    rightTrigger = new AnalogTrigger(codriverController, Axis.RT, 0.5);
    leftTrigger = new AnalogTrigger(codriverController, Axis.LT, 0.5);
    feederRightTrigger = new AnalogTrigger(driverController, Axis.RT, 0.5);
    feederLeftTrigger = new AnalogTrigger(driverController, Axis.LT, 0.5);

    rightBumper = new Keybind(codriverController, Button.RightBumper);
    leftBumper = new Keybind(codriverController, Button.LeftBumper);

    // select modes
    selectButton.trigger().and(startButton.trigger())
        .and(rightTrigger.trigger().and(leftTrigger.trigger()).negate())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Climb)));
    startButton.trigger().and(selectButton.trigger().negate())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Coral)));
    selectButton.trigger().and(startButton.trigger().negate())
        .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Algae)));

    rightStickClick.trigger().negate().and(leftStickClick.trigger())
        .onTrue(new InstantCommand(() -> e.manualCoral = true));
    rightStickClick.trigger().and(leftStickClick.trigger().negate())
        .onTrue(new InstantCommand(() -> e.manualCoral = false));

    // Allow exiting Climb mode
    selectButton.trigger().and(startButton.trigger())
        .and(rightTrigger.trigger().and(leftTrigger.trigger()))
        .whileTrue(new RequesteStateCmd(e, RequestState.UnlockClimb));

    // shoot
    rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new ShootCmd(e));
    rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new ShootCmd(e));
    leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new SuckCmd(e));

    // snap feeder station angles
    feederRightTrigger.trigger()
        .whileTrue(new InstantCommand(() -> snap = SnapButton.LeftFeeder));
    feederLeftTrigger.trigger()
        .whileTrue(new InstantCommand(() -> snap = SnapButton.RightFeeder));

    // snap to reef angles
    snapA.trigger().and(
        snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate())))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefF));
    snapY.trigger().and(
        snapB.trigger().negate().and(snapX.trigger().negate().and(snapA.trigger().negate())))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefB));

    // for b and x allow just button or combo a
    snapB.trigger().and(snapX.trigger().negate().and(snapY.trigger().negate()))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefFR));
    snapX.trigger().and(snapB.trigger().negate().and(snapY.trigger().negate()))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefFL));

    // combo buttons!
    snapB.trigger().and(snapY.trigger().and(snapX.trigger().negate().and(snapA.trigger().negate())))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBR));
    snapX.trigger().and(snapY.trigger().and(snapB.trigger().negate().and(snapA.trigger().negate())))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBL));

    snapA.trigger().negate()
        .and(snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate()
            .and(feederLeftTrigger.trigger().negate()
                .and(feederRightTrigger.trigger().negate())))))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.None));

    // request states for elevclarm
    // include negative feedback (rumble) for unavailable changes of state/mode TODO

    // Unjam strats
    leftBumper.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat1));
    rightBumper.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat2));

    // go to lvl 1
    aButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel1));
    // go to lvl 2
    bButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel2));
    // go to lvl 3
    xButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel3));
    // go to lvl 4
    yButton.trigger().and(() -> e.getEMode() == ControlMode.Coral)
        .whileTrue(new RequesteStateCmd(e, RequestState.CoralLevel4));

    // algae position bottom
    aButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.AlgaeBottom));
    // algae position top
    bButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.AlgaeTop));
    // barge
    yButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.Barge));
    // processor
    xButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.Processor));

    climb.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbState.Up));
    unclimb.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbState.Down));

    leftDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftX = -0.08));
    rightDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftX = 0.08));
    upDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftY = -0.08));
    downDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftY = 0.08));

    leftDpad.trigger().negate().and(rightDpad.trigger().negate())
        .whileTrue(new InstantCommand(() -> dpadShiftX = 0));
    upDpad.trigger().negate().and(downDpad.trigger().negate())
        .whileTrue(new InstantCommand(() -> dpadShiftY = 0));

    driverController.leftBumper().and(driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> snap = SnapButton.Left));
    driverController.rightBumper().and(driverController.leftBumper().negate())
        .onTrue(new InstantCommand(() -> snap = SnapButton.Right));
    driverController.leftBumper().and(driverController.rightBumper())
        .whileTrue(new InstantCommand(() -> snap = SnapButton.Center))
        .onFalse(new InstantCommand(() -> {
          // do something to drive backwards for 0.4 seconds TODO doo doo
          backItUpTimer.restart();
        }));
    driverController.leftBumper().negate().and(driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> snap = SnapButton.None));

    driverController.start().and(driverController.back().negate()).onTrue(new InstantCommand(() -> {
      if (Robot.alliance == Alliance.Red) {
        drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
      } else {
        drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
      }
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
      drivetrain.resetPose(llMeasurement.pose);
    }));
    // driverController.start().onTrue(drivetrain.runOnce(() ->
    // drivetrain.seedFieldCentric()));

    driveFCFA.HeadingController.setPID(7, 0, 0);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // (On blue side. On red, Y is right and X +ve is away from blue)
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          if (climber.climbMotor.getPosition() < -140) {
            // point wheels to 0 when climbed for easier manipulation post match
            return point.withModuleDirection(Rotation2d.fromDegrees(0));
          }
          if (e.rightElevatorMotor.getPosition() > 10) {
            heightFactor = (1.4 / e.rightElevatorMotor.getPosition()) * 15;
            if (heightFactor < 0.2) {
              heightFactor = 0.2;
            }
          } else {
            heightFactor = 1;
          }

          double rotate = ExtraMath.deadzone(
              -driverController.getRightX() * heightFactor * MaxAngularRate,
              0.1);
          double horizontal = ExtraMath.deadzone(
              -driverController.getLeftX() * heightFactor * MaxSpeed, 0.1);
          double vertical = ExtraMath.deadzone(
              -driverController.getLeftY() * heightFactor * MaxSpeed, 0.1);

          // limelight snaps
          if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center) {
            double tx, ty, yaw;
            double targetTx, targetTy = 0.580, targetYaw = 0; // define unchanging values
            double[][] camRet;
            double[] botPose = null;

            String primaryCam, fallbackCam;

            switch (snap) {
              case Right:
                primaryCam = "limelight-left";
                fallbackCam = "limelight-right";
                targetTx = 0.150;
                // targetTy = 0.587;
                // targetYaw = 0;
                break;
              case Left:
                primaryCam = "limelight-right";
                fallbackCam = "limelight-left";
                targetTx = -0.18;
                break;
              default:
                primaryCam = "limelight-left";
                fallbackCam = "limelight-right";
                targetTx = 0.0;
                targetTy = 0.65;
                if (e.getEMode() == ControlMode.Algae) {
                  targetTx = 0.0;
                  targetTy = 0.49;
                }
            }

            targetTx = targetTx + dpadShiftX;
            targetTy = targetTy + dpadShiftY;

            idToLookFor = getFacingSide().getTag();
            camRet = OURLimelightHelpers.getBotPoseTargetSpace(primaryCam, fallbackCam, idToLookFor);
            if (camRet != null) {
              botPose = camRet[0];
            }

            if (botPose != null) {
              tx = botPose[0]; // meters
              ty = -botPose[2]; // meters - secretly grabbing tz - away is
                                // more negative
              yaw = botPose[4]; // degrees

              // vector's represent needed movement from the robot to the tag
              // in targetspace
              double vectorX = targetTx - tx;
              double vectorY = targetTy - ty;
              double vectorYaw = targetYaw - yaw;

              double pt = 2; // translation p value
              double pr = 0.05; // rotation p

              SmartDashboard.putNumber("Velocity X", ExtraMath.clampedDeadzone(vectorY * -pt, 1, .03));
              SmartDashboard.putNumber("Velocity Y", ExtraMath.clampedDeadzone(vectorX * -pt, 1, .03));
              SmartDashboard.putNumber("Rotation", ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .08));
              // Y goes in X and X goes in y because of comment above
              // setDefaultCommand
              return driveRC.withVelocityX(ExtraMath.clampedDeadzone(vectorY * -pt, 1, .03))
                  .withVelocityY(ExtraMath.clampedDeadzone(vectorX * -pt, 1, .03))
                  .withRotationalRate(ExtraMath.clampedDeadzone(vectorYaw * -pr, 1, .08));
              
            } else {
              // womp womp good enough
              return driveFC.withVelocityX(vertical)
                  .withVelocityY(horizontal)
                  .withRotationalRate(rotate);
            }
          } else {
            int[] ids = {};
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight-left", ids);
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight-right", ids);
            // field snaps
            if (snap != SnapButton.None) {
              direction = switch (snap) {
                case LeftFeeder -> RightFeederAngle;
                case RightFeeder -> LeftFeederAngle;
                case ReefB -> ReefBAngle;
                case ReefBL -> ReefBLAngle;
                case ReefBR -> ReefBRAngle;
                case ReefFL -> ReefFLAngle;
                case ReefFR -> ReefFRAngle;
                case ReefF -> ReefFAngle;
                default -> ReefFAngle;
              };

              return driveFCFA.withVelocityX(vertical)
                  .withVelocityY(horizontal)
                  .withTargetDirection(direction);
            } else {
              SmartDashboard.putNumber("joystic velocity", vertical);
              // If we just released the center alignment in algae mode... drive back 0.4 seconds TODO
              if(backItUpTimer.get() < 0.4 && e.getEMode() == ControlMode.Algae){
                return driveRC.withVelocityX(-1.5) // ExtraMath.deadzone(-driverController.getLeftY() * heightFactor * MaxSpeed, 0.1);
                    .withVelocityY(horizontal)
                    .withRotationalRate(rotate);
              } else{
                return driveFC.withVelocityX(vertical)
                    .withVelocityY(horizontal)
                    .withRotationalRate(rotate);
              }
            }
          }
        }));

    drivetrain.registerTelemetry(logger::telemeterize);
  }
  
  public ReefSide getFacingSide() {
    boolean isOnRed = Robot.alliance == Alliance.Red;
    double angle = drivetrain.getState().Pose.getRotation().getDegrees();
    if (isOnRed) {
      angle = (angle + 180) % 360;
    }
    if (330 <= angle || angle < 30) {
      return ReefSide.FC;
    } else if (30 <= angle && angle < 90) {
      return ReefSide.FR;
    } else if (90 <= angle && angle < 150) {
      return ReefSide.BR;
    } else if (150 <= angle && angle < 210) {
      return ReefSide.BC;
    } else if (210 <= angle && angle < 270) {
      return ReefSide.BL;
    } else if (270 <= angle && angle < 330) {
      return ReefSide.FL;
    } else {
      // This should never happen, but if it does, don't fail
      System.out.println("Invalid angle in getFacingSide");
      System.out.println(angle);
      return ReefSide.FC;
    }
}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

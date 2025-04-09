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
import frc.robot.commands.auto.Left3Piece;
import frc.robot.commands.auto.Left4Piece;
import frc.robot.commands.auto.Right3Piece;
import frc.robot.commands.auto.TautoFLF;
import frc.robot.commands.auto.TautoLB;
import frc.robot.commands.auto.TautoLF;
import frc.robot.commands.auto.TautoRB;
import frc.robot.commands.auto.TautoRF;
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
import frc.robot.subsystems.ClimberSubsystem.ClimbHeight;
import frc.robot.subsystems.ClimberSubsystem.ManualClimbState;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.LEDSubsystem;

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
  public LEDSubsystem led;
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

  DPadButton coDpadUp;
  DPadButton coDpadDown;
  DPadButton coDpadLeft;
  DPadButton coDpadRight;

  public double dpadShiftX;
  public double dpadShiftY;

  // shoot keybind
  AnalogTrigger rightTrigger;
  AnalogTrigger leftTrigger;
  AnalogTrigger feederRightTrigger;
  AnalogTrigger feederLeftTrigger;

  Keybind rightBumper;
  Keybind leftBumper;

  public SnapButton snap = SnapButton.None;

  double speedFactor = 1.0;

  

  static Rotation2d LeftFeederAngle = Rotation2d.fromDegrees(90 - 144.011);
  static Rotation2d RightFeederAngle = Rotation2d.fromDegrees(144.011 - 90); // measurements stolen from spectrum
  static Rotation2d ReefFAngle = Rotation2d.fromDegrees(0);
  static Rotation2d LeftL1Angle = Rotation2d.fromDegrees(45);
  static Rotation2d RightL1Angle = Rotation2d.fromDegrees(-45);
  static Rotation2d ReefFRAngle = Rotation2d.fromDegrees(60);
  static Rotation2d ReefFLAngle = Rotation2d.fromDegrees(-60);
  static Rotation2d ReefBLAngle = Rotation2d.fromDegrees(-120);
  static Rotation2d ReefBRAngle = Rotation2d.fromDegrees(120);
  static Rotation2d ReefBAngle = Rotation2d.fromDegrees(180);
  static Rotation2d ProcessorAngle = Rotation2d.fromDegrees(-90);
  
  Rotation2d direction;
  // kSpeedAt12Volts desired top speed
  private final double MaxSpeed;
  // 3/4 of a rotation per second max angular velocity
  private final double MaxAngularRate;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
      // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

  final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
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

  private final Telemetry logger;

  public final CommandSwerveDrivetrain drivetrain;
  double heightFactor;

  final SendableChooser<AutoCommand> autoChooser;

  public Timer backItUpTimer;
  // public Timer jerkTimer;
  public double positionError = Double.MAX_VALUE;

  public RobotContainer() {
    // Initialize the drivetrain first, so it runs periodic first
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

    // gyro = new PigeonSubsystem();
    pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);
    e = new ElevClArmSubsystem();
    climber = new ClimberSubsystem();
    led = new LEDSubsystem(this);

    backItUpTimer = new Timer();
    // jerkTimer = new Timer();
    backItUpTimer.start();

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    // autoChooser.addOption("Testing", new Testing(e, drivetrain, driveRC));
    autoChooser.addOption("CenterAuto", new CenterAuto(this, e, drivetrain, driveRC));
    autoChooser.addOption("Left3PieceAuto", new Left3Piece(this, e, drivetrain, driveRC));
    autoChooser.addOption("Right3PieceAuto", new Right3Piece(this, e, drivetrain, driveRC));
    autoChooser.addOption("Left4PieceAuto", new Left4Piece(this, e, drivetrain, driveRC));

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

    coDpadUp = new DPadButton(codriverController, DPad.Up);
    coDpadDown = new DPadButton(codriverController, DPad.Down);
    coDpadLeft = new DPadButton(codriverController, DPad.Left);
    coDpadRight = new DPadButton(codriverController, DPad.Right);

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
    feederRightTrigger.trigger().and(driverController.rightBumper().negate())
        .whileTrue(new InstantCommand(() -> snap = SnapButton.LeftFeeder));
    feederLeftTrigger.trigger().and(driverController.leftBumper().negate())
        .whileTrue(new InstantCommand(() -> snap = SnapButton.RightFeeder));

    // L1 snap
    driverController.rightBumper().and(feederRightTrigger.trigger())
        .whileTrue(new InstantCommand(() -> snap = SnapButton.RightL1));
    driverController.leftBumper().and(feederLeftTrigger.trigger())
        .whileTrue(new InstantCommand(() -> snap = SnapButton.LeftL1));

    // snap to reef angles
    // snapA.trigger().and(
    //     snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate())))
    //     .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefF));
    snapY.trigger().and(
        snapB.trigger().negate().and(snapX.trigger().negate().and(snapA.trigger().negate())))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefB));

    // for b and x allow just button or combo a
    snapB.trigger().and(snapX.trigger().negate().and(snapY.trigger().negate()))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.Processor));
    // snapX.trigger().and(snapB.trigger().negate().and(snapY.trigger().negate()))
    //     .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefFL));

    // // combo buttons!
    // snapB.trigger().and(snapY.trigger().and(snapX.trigger().negate().and(snapA.trigger().negate())))
    //     .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBR));
    // snapX.trigger().and(snapY.trigger().and(snapB.trigger().negate().and(snapA.trigger().negate())))
    //     .whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBL));

    // When nothing pressed, don't snap!
    snapA.trigger().negate()
        .and(snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate()
            .and(feederLeftTrigger.trigger().negate()
                .and(feederRightTrigger.trigger().negate())))))
        .whileTrue(new InstantCommand(() -> snap = SnapButton.None));

    // request states for elevclarm
    // include negative feedback (rumble) for unavailable changes of state/mode TODO

    // // Unjam strats
    // leftBumper.trigger().and(() -> e.getEMode() == ControlMode.Coral)
    //     .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat1));
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
    yButton.trigger().and(leftTrigger.trigger().negate().and(() -> e.getEMode() == ControlMode.Algae))
        .whileTrue(new RequesteStateCmd(e, RequestState.Barge));
    // bargeplace
    yButton.trigger().and(leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Algae))
        .whileTrue(new RequesteStateCmd(e, RequestState.BargePlace));
    // processor
    xButton.trigger().and(() -> e.getEMode() == ControlMode.Algae)
        .whileTrue(new RequesteStateCmd(e, RequestState.Processor));

    coDpadUp.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ManualClimbState.Up));
    coDpadDown.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ManualClimbState.Down));
    coDpadLeft.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbHeight.Stowed));
    coDpadRight.trigger().and(() -> e.getEMode() == ControlMode.Climb)
        .whileTrue(new ClimberCmd(climber, ClimbHeight.Engaged));

    // coDpadUp.trigger().and(() -> e.getEMode() == ControlMode.Coral)
    //     .whileTrue(new TautoRF(this, e, drivetrain, driveRC));
    coDpadLeft.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger().negate())
        .whileTrue(new TautoRF(this, e, drivetrain, driveRC));
    coDpadRight.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger().negate())
        .whileTrue(new TautoRF(this, e, drivetrain, driveRC));
    coDpadDown.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger().negate())
        .whileTrue(new TautoFLF(this, e, drivetrain, driveRC));
    coDpadUp.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger().negate())
        .whileTrue(new TautoRB(this, e, drivetrain, driveRC));

    coDpadLeft.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger())
        .whileTrue(new TautoLF(this, e, drivetrain, driveRC));
    coDpadRight.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger())
        .whileTrue(new TautoLF(this, e, drivetrain, driveRC));
    coDpadDown.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger())
        .whileTrue(new TautoFLF(this, e, drivetrain, driveRC));
    coDpadUp.trigger().and(() -> e.getEMode() == ControlMode.Coral).and(leftBumper.trigger())
        .whileTrue(new TautoLB(this, e, drivetrain, driveRC));

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
          backItUpTimer.restart();
        }));
        driverController.leftBumper().negate().and(driverController.rightBumper().negate())
        .onTrue(new InstantCommand(() -> snap = SnapButton.None));
        
        driverController.start().and(driverController.back().negate()).onTrue(new InstantCommand(() -> {
          updateDrivetrainRobotPerspective();
          var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
          if (llMeasurement != null){
            drivetrain.resetPose(llMeasurement.pose);
          }
        }));

        // best solution ever trust
        // driverController.x()
        //     .onTrue(new InstantCommand(() -> jerkTimer.start()));

        // driverController.start().onTrue(drivetrain.runOnce(() ->
        // drivetrain.seedFieldCentric()));
        
    driveFCFA.HeadingController.setPID(7, 0, 0);
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // (On blue side. On red, Y is right and X +ve is away from blue)
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
          // if(jerkTimer.get() != 0 && jerkTimer.get() < 0.1){
          //   return driveFC.withVelocityX(0)
          //   .withVelocityY(0)
          //   .withRotationalRate(0);
          // } else if (jerkTimer.get() > 0.1){
          //   jerkTimer.stop();
          //   jerkTimer.reset();
          // }
          // if (climber.climbMotor.getPosition() < -140) {
          //   // point wheels to 0 when climbed for easier manipulation post match
          //   return point.withModuleDirection(Rotation2d.fromDegrees(0));
          // }
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
          if (snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center || snap == SnapButton.LeftL1 || snap == SnapButton.RightL1) {
            double tx, ty, yaw;
            double targetTx, targetTy = 0.580, targetYaw = 0; // define unchanging values
            double[][] camRet;
            double[] botPose = null;

            String primaryCam, fallbackCam;

            switch (snap) {
              case Right:
                primaryCam = "limelight-left";
                fallbackCam = "limelight-right";
                targetTx = 0.170; // competition value
                // targetTx = 0.150;
                // targetTy = 0.587;
                // targetYaw = 0;
                break;
              case Left:
                primaryCam = "limelight-right";
                fallbackCam = "limelight-left";
                targetTx = -0.170; // competition value
                // targetTx = -0.18;
                break;
              case RightL1:
                primaryCam = "limelight-left";
                fallbackCam = "limelight-right";
                targetTx = -0.05; // competition value
                targetTy = 0.70;
                targetYaw = 25;
                break;
              case LeftL1:
                primaryCam = "limelight-right";
                fallbackCam = "limelight-left";
                targetTx = 0.05; // competition value
                targetTy = 0.70;
                targetYaw = -25;
                break;
              default:
                primaryCam = "limelight-left";
                fallbackCam = "limelight-right";
                targetTx = 0.0;
                targetTy = 0.45;
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

            if (botPose != null && botPose.length >= 5) {
              tx = botPose[0]; // meters
              ty = -botPose[2]; // meters - secretly grabbing tz - away is
                                // more negative
              yaw = botPose[4]; // degrees

              // vector's represent needed movement from the robot to the tag
              // in targetspace
              double vectorX = targetTx - tx;
              double vectorY = targetTy - ty;
              double vectorYaw = targetYaw - yaw;

              positionError = Math.sqrt((vectorX * vectorX) + (vectorY * vectorY));

              // Reset odometry when we successfully align with the reef
              // var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(primaryCam);
              // if (llMeasurement != null) {
              //   drivetrain.resetPose(llMeasurement.pose);
              // }

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
                case RightL1 -> RightL1Angle;
                case LeftL1 -> LeftL1Angle;
                case Processor -> ProcessorAngle;
                default -> ReefFAngle;
              };

              return driveFCFA.withVelocityX(vertical)
                  .withVelocityY(horizontal)
                  .withTargetDirection(direction);
            } else {
              // If we just released the center alignment in algae mode... drive back 0.3 seconds TODO
              if(backItUpTimer.get() < 0.3 && e.getEMode() == ControlMode.Algae){
                return driveRC.withVelocityX(-1.5); // ExtraMath.deadzone(-driverController.getLeftY() * heightFactor * MaxSpeed, 0.1);
                    // .withVelocityY(horizontal)
                    // .withRotationalRate(rotate);
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
    } else{
      angle = (angle + 360) % 360;
    }
    // SmartDashboard.putNumber("angle", angle);
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstants;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.DPadButton;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.DPadButton.DPad;
import frc.robot.input.Keybind;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PigeonSubsystem;

public class RobotContainer {
        public final CommandXboxController driverController = new CommandXboxController(
                        Constants.DRIVER_CONTROLLER_PORT);
        public final CommandXboxController codriverController = new CommandXboxController(
                        Constants.CODRIVER_CONTROLLER_PORT);
        // public final CommandXboxController testController = new CommandXboxController(Constants.TEST_CONTROLLER_PORT);

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

        DPadButton rightDpad;
        DPadButton leftDpad;
        DPadButton upDpad;
        DPadButton downDpad;

        double dpadShiftX;
        double dpadShiftY;

        // shoot keybind
        AnalogTrigger rightTrigger;
        AnalogTrigger leftTrigger;
        AnalogTrigger feederRightTrigger;
        AnalogTrigger feederLeftTrigger;

        enum SnapButton {
                Left, Right, Center, LeftFeeder, RightFeeder, ReefF, ReefFR, ReefFL, ReefBL, ReefBR, ReefB, None
        }
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

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * speedFactor; // kSpeedAt12Volts
                                                                                                    // desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * speedFactor; // 3/4 of a
                                                                                                        // rotation per
                                                                                                        // second
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
                        // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

        private final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
                        // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.FieldCentricFacingAngle driveFCFA = new SwerveRequest.FieldCentricFacingAngle()
                        // .withDeadband(0.0).withRotationalDeadband(0.0) // deadband added later
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);
        
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        double heightFactor;

        public RobotContainer() {
                gyro = new PigeonSubsystem();
                pdp = new PowerDistribution(Constants.PDP_ID, ModuleType.kCTRE);

                e = new ElevClArmSubsystem();
                climber = new ClimberSubsystem();
                led = new LEDSubsystem(this);

                // digitalio = new DigitalIOSubsystem(arm, shooter, floorIntake, climber); // if
                // adam wants buttons again

                configureBindings();
        }

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

                leftDpad = new DPadButton(driverController, DPad.Left);
                rightDpad = new DPadButton(driverController, DPad.Right);
                upDpad = new DPadButton(driverController, DPad.Up);
                downDpad = new DPadButton(driverController, DPad.Down);

                // processor (just shoot in safe?) maybe default to processor rather than algaesafe
                rightTrigger = new AnalogTrigger(codriverController, Axis.RT, 0.5);
                leftTrigger = new AnalogTrigger(codriverController, Axis.LT, 0.5);
                feederRightTrigger = new AnalogTrigger(driverController, Axis.RT, 0.5);
                feederLeftTrigger = new AnalogTrigger(driverController, Axis.LT, 0.5);
                

                // select modes
                // selectButton.trigger().and(startButton.trigger())
                //                 .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Climb)));
                startButton.trigger().and(selectButton.trigger().negate())
                                .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Coral)));
                selectButton.trigger().and(startButton.trigger().negate())
                                .onTrue(new InstantCommand(() -> e.requestMode(ControlMode.Algae)));

                // shoot
                rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
                                .whileTrue(new ShootCmd(e));
                rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Algae)
                                .whileTrue(new ShootCmd(e));

                // climb up and down with modes for one controller
                // rightTrigger.trigger().and(() -> e.getEMode() == ControlMode.Climb)
                //                 .whileTrue(new ClimberCmd(climber, ClimbState.Down));
                // leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Climb)
                //                 .whileTrue(new ClimberCmd(climber, ClimbState.Up));
                feederRightTrigger.trigger()
                                .whileTrue(new InstantCommand(() -> snap = SnapButton.LeftFeeder));
                feederLeftTrigger.trigger()
                                .whileTrue(new InstantCommand(() -> snap = SnapButton.RightFeeder));
                
                
                snapA.trigger().and(snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate()
                        ))).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefF));
                snapY.trigger().and(snapB.trigger().negate().and(snapX.trigger().negate().and(snapA.trigger().negate()
                        ))).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefB));

                // for b and x allow just button or combo a
                snapB.trigger().and(snapX.trigger().negate().and(snapY.trigger().negate() 
                        )).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefFR));
                snapX.trigger().and(snapB.trigger().negate().and(snapY.trigger().negate()
                        )).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefFL));

                // combo buttons!
                snapB.trigger().and(snapY.trigger().and(snapX.trigger().negate().and(snapA.trigger().negate()
                        ))).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBR));
                snapX.trigger().and(snapY.trigger().and(snapB.trigger().negate().and(snapA.trigger().negate()
                        ))).whileTrue(new InstantCommand(() -> snap = SnapButton.ReefBL));

                snapA.trigger().negate().and(snapB.trigger().negate().and(snapX.trigger().negate().and(snapY.trigger().negate().and(feederLeftTrigger.trigger().negate().and(feederRightTrigger.trigger().negate()
                        ))))).whileTrue(new InstantCommand(() -> snap = SnapButton.None));
                

                // request states for elevclarm
                // include negative feedback (rumble) for unavailable changes of state/mode

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

                leftDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftX = -0.05));
                rightDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftX = 0.05));
                upDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftY = -0.05));
                downDpad.trigger().whileTrue(new InstantCommand(() -> dpadShiftY = 0.05));

                leftDpad.trigger().negate().and(rightDpad.trigger().negate()).whileTrue(new InstantCommand(() -> dpadShiftX = 0));
                upDpad.trigger().negate().and(downDpad.trigger().negate()).whileTrue(new InstantCommand(() -> dpadShiftY = 0));
                
                // unjam
                leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
                .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat1));
                
                driverController.leftBumper().and(driverController.rightBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.Left));
                driverController.rightBumper().and(driverController.leftBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.Right));
                driverController.leftBumper().and(driverController.rightBumper()).onTrue(new InstantCommand(() -> snap = SnapButton.Center));
                driverController.leftBumper().negate().and(driverController.rightBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.None));
                
                driverController.start().and(driverController.back().negate()).onTrue(new InstantCommand(() -> {
                        Alliance alliance = DriverStation.getAlliance().orElse(null);
                        if (alliance == Alliance.Red) {
                                drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
                        } else {
                                drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
                        }
                        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
                        drivetrain.resetPose(llMeasurement.pose);
                }));
                // driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                
                driveFCFA.HeadingController.setPID(7, 0, 0);
                
                
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                        // Drivetrain will execute this command periodically
                        drivetrain.applyRequest(() -> {
                                        //limelight snaps
                                        if(snap == SnapButton.Right || snap == SnapButton.Left || snap == SnapButton.Center){
                                                double tx,ty,yaw;
                                                double targetTx,targetTy = 0.587, targetYaw = 0; // define unchanging values
                                                double[] botPose;

                                                String primaryCam, fallbackCam;

                                                switch (snap) {
                                                        case Right:
                                                                primaryCam = "limelight-left";
                                                                fallbackCam = "limelight-right";
                                                                targetTx = 0.160;
                                                                // targetTy = 0.587;
                                                                // targetYaw = 0;
                                                                break;
                                                        case Left:
                                                                primaryCam = "limelight-right";
                                                                fallbackCam = "limelight-left";
                                                                targetTx = -0.17;
                                                                break;
                                                        default:
                                                                primaryCam = "limelight-left";
                                                                fallbackCam = "limelight-right";
                                                                targetTx = 0.0;
                                                }

                                                targetTx = targetTx + dpadShiftX;
                                                targetTy = targetTy + dpadShiftY;

                                                botPose = getValidBotPose(primaryCam, fallbackCam);

                                                if(botPose != null){
                                                        tx = botPose[0]; // meters
                                                        ty = -botPose[2]; // meters - secretly grabbing tz - away is more negative
                                                        yaw = botPose[4]; // degrees

                                                        
                                                        // vector's represent needed movement from the robot to the tag in targetspace
                                                        double vectorX = targetTx-tx;
                                                        double vectorY = targetTy-ty;
                                                        double vectorYaw = targetYaw-yaw;
                                                        
                                                        double pt = 2.5; // translation p value
                                                        double pr = 0.1; // rotation p
                                                        
                                                        // Y goes in X and X goes in y because of comment above setDefaultCommand
                                                        return driveRC.withVelocityX(clampedDeadzone(vectorY * -pt, 1, .03)) // Drive
                                                        .withVelocityY(clampedDeadzone(vectorX * -pt, 1, .03))
                                                        .withRotationalRate(clampedDeadzone(vectorYaw * -pr, 1, .1));
                                                } else {
                                                        return driveFC.withVelocityX(0) // Drive
                                                        .withVelocityY(0)
                                                        .withRotationalRate(0);
                                                }
                                        } else {
                                                if(e.rightElevatorMotor.getPosition() > 10){
                                                        heightFactor = (1/e.rightElevatorMotor.getPosition()) * 15;
                                                        if(heightFactor < 0.1){
                                                                heightFactor = 0.1;
                                                        }
                                                } else {
                                                        heightFactor = 1;
                                                }

                                                double rotate = ExtraMath.deadzone(-driverController.getRightX() * heightFactor * MaxAngularRate, 0.1);
                                                double horizontal = ExtraMath.deadzone(-driverController.getLeftX() * heightFactor * MaxSpeed, 0.1);
                                                double vertical = ExtraMath.deadzone(-driverController.getLeftY() * heightFactor * MaxSpeed, 0.1);
                                                // field snaps
                                                if(snap != SnapButton.None){
                                                        switch (snap) {
                                                                case LeftFeeder:
                                                                        direction = RightFeederAngle;
                                                                        break;
                                                                case RightFeeder:
                                                                        direction = LeftFeederAngle;
                                                                        break;
                                                                case ReefB:
                                                                        direction = ReefBAngle;
                                                                        break;
                                                                case ReefBL:
                                                                        direction = ReefBLAngle;
                                                                        break;
                                                                case ReefBR:
                                                                        direction = ReefBRAngle;
                                                                        break;
                                                                case ReefFL:
                                                                        direction = ReefFLAngle;
                                                                        break;
                                                                case ReefFR:
                                                                        direction = ReefFRAngle;
                                                                        break;
                                                                default:
                                                                case ReefF:
                                                                        direction = ReefFAngle;
                                                                        break;
                                                        }

                                                        SmartDashboard.putString("direction", direction.toString());

                                                        return driveFCFA.withVelocityX(vertical) // Drive with rotation2d
                                                                .withVelocityY(horizontal)
                                                                .withTargetDirection(direction);
                                                } else {
                                                        return driveFC.withVelocityX(vertical) // Drive with stick rotation
                                                                .withVelocityY(horizontal)
                                                                .withRotationalRate(rotate);
                                                }
                                        }
                                }));

                // it's drivin time
                // Load the path we want to pathfind to and follow
                // PathPlannerPath path = null;
                // try {
                //         path = PathPlannerPath.fromPathFile("testy");
                // } catch (FileVersionException e1) {
                //         // TODO Auto-generated catch block
                //         System.exit(1);
                //         e1.printStackTrace();
                // } catch (IOException e1) {
                //         System.exit(1);
                //         // TODO Auto-generated catch block
                //         e1.printStackTrace();
                // } catch (ParseException e1) {
                //         System.exit(1);
                //         // TODO Auto-generated catch block
                //         e1.printStackTrace();
                // }

                // // Create the constraints to use while pathfinding. The constraints defined in
                // // the path will only be used for the path.
                // PathConstraints constraints = new PathConstraints(
                //                 2, 2,
                //                 Units.degreesToRadians(540), Units.degreesToRadians(720));

                // Pose2d targetPose = new Pose2d(6.198, 4.007, Rotation2d.fromDegrees(180));
                // // Since AutoBuilder is configured, we can use it to build pathfinding commands
                // Command testyCommand = AutoBuilder.pathfindToPose(
                //                 targetPose,
                //                 constraints, 0.0);

                // codriverController.start().whileTrue(testyCommand);


                // reset the field-centric heading on start
                // start is the right menu button

                drivetrain.registerTelemetry(logger::telemeterize);
                // reg drive
                // snap to reef left
                // snap to reef right controls TODO

                // Drive, Limelight
        }

        /**
         * Applies a deadzone to the input value, then clamps it within the specified min and max range.
         *
         * @param value    The input value to be processed.
         * @param min      The minimum allowed value after clamping.
         * @param max      The maximum allowed value after clamping.
         * @param deadzone The threshold below which the value is set to zero.
         * @return The adjusted value after applying the deadzone and clamping.
         */
        private double clampedDeadzone(double value, double min, double max, double deadzone) {
                return ExtraMath.clamp(ExtraMath.deadzone(value, deadzone), min, max);
        }
        
        /**
         * Overload that assumes a symmetric clamping range.
         * The value is clamped within [-amp, amp] after applying the deadzone.
         *
         * @param value    The input value to be processed.
         * @param amp      The absolute maximum amplitude for clamping (range: -amp to amp).
         * @param deadzone The threshold below which the value is set to zero.
         * @return The adjusted value after applying the deadzone and clamping.
         */
        private double clampedDeadzone(double value, double amp, double deadzone) {
                return ExtraMath.clamp(ExtraMath.deadzone(value, deadzone), -amp, amp);
        }
    

        private double[] getValidBotPose(String primary, String fallback) {
                double[] botPose = LimelightHelpers.getBotPose_TargetSpace(primary);
                if(doesBotPoseExist(botPose)){
                        return botPose;
                }
                botPose = LimelightHelpers.getBotPose_TargetSpace(fallback);
                if(doesBotPoseExist(botPose)){
                        return botPose;
                }
                return null;
        }

        boolean doesBotPoseExist(double[] botPose) {
                for (double value : botPose) {
                    if (value != 0.0) {
                        return true;
                    }
                }
                return false;
        }
            

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}

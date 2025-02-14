// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.RequesteStateCmd;
import frc.robot.commands.ShootCmd;
import frc.robot.drive.CommandSwerveDrivetrain;
import frc.robot.drive.ProfiledFieldCentricFacingAngle;
import frc.robot.drive.Telemetry;
import frc.robot.drive.TunerConstants;
import frc.robot.input.AnalogTrigger;
import frc.robot.input.Keybind;
import frc.robot.input.AnalogTrigger.Axis;
import frc.robot.input.Keybind.Button;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevClArmSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PigeonSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ElevClArmSubsystem.ControlMode;
import frc.robot.subsystems.ElevClArmSubsystem.RequestState;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.drive.TunerConstants;
import frc.robot.drive.CommandSwerveDrivetrain;

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

        // shoot keybind
        AnalogTrigger rightTrigger;
        AnalogTrigger leftTrigger;
        AnalogTrigger climbTrigger;
        AnalogTrigger unclimbTrigger;

        enum DriveMode {
                DriverControlled, Limelight
        }
        enum SnapButton {
                Left, Right, Center, None
        }
        DriveMode driveMode = DriveMode.DriverControlled;
        SnapButton snap = SnapButton.None;


        double speedFactor = 1.0;

        Rotation2d direction = new Rotation2d();

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * speedFactor; // kSpeedAt12Volts
                                                                                                    // desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * speedFactor; // 3/4 of a
                                                                                                        // rotation per
                                                                                                        // second
        // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric driveFC = new SwerveRequest.FieldCentric()
                        .withDeadband(0.0).withRotationalDeadband(0.0) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive

        private final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
                        .withDeadband(0.0).withRotationalDeadband(0.0) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.FieldCentricFacingAngle driveFCFA = new SwerveRequest.FieldCentricFacingAngle()
                        .withDeadband(0.0).withRotationalDeadband(0.0) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors

        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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

                // processor (just shoot in safe?) maybe default to processor rather than algaesafe
                rightTrigger = new AnalogTrigger(codriverController, Axis.RT, 0.5);
                leftTrigger = new AnalogTrigger(codriverController, Axis.LT, 0.5);
                climbTrigger = new AnalogTrigger(driverController, Axis.RT, 0.5);
                unclimbTrigger = new AnalogTrigger(driverController, Axis.LT, 0.5);
                

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
                climbTrigger.trigger()
                                .whileTrue(new ClimberCmd(climber, ClimbState.Down));
                unclimbTrigger.trigger()
                                .whileTrue(new ClimberCmd(climber, ClimbState.Up));

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
                
                // unjam
                leftTrigger.trigger().and(() -> e.getEMode() == ControlMode.Coral)
                .whileTrue(new RequesteStateCmd(e, RequestState.UnjamStrat1));
                
                driverController.leftBumper().and(driverController.rightBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.Left));
                driverController.rightBumper().and(driverController.leftBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.Right));
                driverController.leftBumper().and(driverController.rightBumper()).onTrue(new InstantCommand(() -> snap = SnapButton.Center));
                driverController.leftBumper().negate().and(driverController.rightBumper().negate()).onTrue(new InstantCommand(() -> snap = SnapButton.None));
                

                driveFCFA.HeadingController.setPID(7, 0, 0);
                
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically

                                drivetrain.applyRequest(() -> {
                                        if(snap != SnapButton.None){
                                                double tx,ty,yaw;
                                                double targetTx,targetTy,targetYaw;
                                                double[] botPose;

                                                if(snap == SnapButton.Right){
                                                        botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
                                                        targetTx = 0.160;
                                                        targetTy = 0.587;
                                                        targetYaw = 0.0;
                                                        
                                                } else if (snap == SnapButton.Left){
                                                        botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
                                                        targetTx = -0.17;
                                                        targetTy = 0.587;
                                                        targetYaw = 0.0;
                                                } else {
                                                        botPose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
                                                        targetTx = 0.0;
                                                        targetTy = 0.587;
                                                        targetYaw = 0.0;
                                                }
                                                tx = botPose[0]; // meters
                                                ty = -botPose[2]; // meters - secretly grabbing tz - away is more negative
                                                yaw = botPose[4]; // degrees
                                                
                                                double vectorX = targetTx-tx;
                                                double vectorY = targetTy-ty;
                                                double vectorYaw = targetYaw-yaw;
                                                double pt = 3.5;
                                                double pr = 0.1;

                                                return driveRC.withVelocityX(ExtraMath.clamp(ExtraMath.deadzone(vectorY * -pt, .05), -0.5, 0.5)) // Drive
                                                .withVelocityY(ExtraMath.clamp(ExtraMath.deadzone(vectorX * -pt, .05), -0.5, 0.5))
                                                .withRotationalRate(ExtraMath.clamp(ExtraMath.deadzone(vectorYaw * -pr,.1), -1, 1));
                                        } else{
                                                double rotate = ExtraMath.deadzone(-driverController.getRightX() * MaxAngularRate, 0.1);
                                                double horizontal = ExtraMath.deadzone(-driverController.getLeftX() * MaxSpeed, 0.1);
                                                double vertical = ExtraMath.deadzone(-driverController.getLeftY() * MaxSpeed, 0.1);

                                                return driveFC.withVelocityX(vertical) // Drive
                                                        .withVelocityY(horizontal)
                                                        .withRotationalRate(rotate);
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
                driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
                // reg drive
                // snap to reef left
                // snap to reef right controls TODO

                // Drive, Limelight
        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}

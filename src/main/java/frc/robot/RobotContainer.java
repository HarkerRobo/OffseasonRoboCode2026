// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.*;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.Swerve;
import frc.robot.commands.climb.ClimbDown;
import frc.robot.commands.climb.ClimbUp;
import frc.robot.commands.climb.Unspool;
import frc.robot.commands.climb.Spool;
import frc.robot.commands.drive.RotateToAngle;
import frc.robot.commands.hood.AimToAngle;
import frc.robot.commands.hood.HoodManual;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.commands.indexer.IndexerStartDefaultSpeed;
import frc.robot.commands.indexer.IndexerStartEjectSpeed;
import frc.robot.commands.indexer.IndexerStartFullSpeed;
import frc.robot.commands.intake.AgitateIntake;
import frc.robot.commands.intake.StartDefaultIntake;
import frc.robot.commands.intake.StartEjectIntake;
import frc.robot.commands.intake.StartRunIntake;
import frc.robot.commands.intakeextension.ExtendIntake;
import frc.robot.commands.intakeextension.RetractIntake;
import frc.robot.commands.shooter.ShooterTargetSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerStartDefaultSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerStartEjectSpeed;
import frc.robot.commands.shooterindexer.ShooterIndexerStartFullSpeed;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.Modules;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeExtension;
import frc.robot.util.Util;

public class RobotContainer 
{
    public enum AlignDirection
    {
        Right,
        Left
    }
    
    public LinearVelocity MaxSpeed = Swerve.SPEED_AT_12_VOLTS.times(1.0); // kSpeedAt12Volts desired top speed
    private AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity
    private AlignDirection alignDirection = AlignDirection.Left;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed.in(MetersPerSecond) * 0.1).withRotationalDeadband(MaxAngularRate.in(RadiansPerSecond) * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public final CommandXboxController driver = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = Modules.createDrivetrain();

    public SendableChooser<Command> testCommandChooser = new SendableChooser<>();
    public ArrayList<SendableChooser<SubsystemStatus>> modeChoosers = new ArrayList<>();

    Function<RobotContainer.AlignDirection, Command> setDirectionFactory = ((AlignDirection direction) ->
        new Command() {
            public void execute () {System.out.println(direction); Robot.instance.robotContainer.setAlignDirection(direction);}
            public boolean isFinished () {return true;}});

    Command alignLeft = setDirectionFactory.apply(AlignDirection.Left);
    Command alignRight = setDirectionFactory.apply(AlignDirection.Right);

    private SendableChooser<Command> autonChooser;
    public static int INTAKE_INDEX = 0;
    public static int INTAKE_EXTENSION_INDEX = 1;
    public static int CLIMB_INDEX = 2;
    public static int HOOD_INDEX = 3;
    public static int HOPPER_INDEX = 4;
    public static int INDEXER_INDEX = 5;
    public static int SHOOTER_INDEX = 6;
    public static int SHOOTER_INDEXER_INDEX = 7;
    public static int INDEXES = 8;

    public enum SubsystemStatus {Enabled, Simulated, Disabled};

    private boolean isSlow = false;
    public boolean mostRecentAim = false; // false = shoot; true = pass
    public boolean isShooting = false;
    public boolean intakeTriggered = false; // true if intake has been enabled
    public boolean intakeExtended = true; //true if intake and hopper have been extended // TODO reverse
    private boolean isRevShoot = true; //true if rev shoot, false if rev pass
  
    public static enum PassDirection {Left, Right, Automatic};

    private PassDirection direction = PassDirection.Automatic; // Default
        
    private Supplier<Command> stow;
    private Supplier<Command> rev;

    /**
     * Sets the desired pass direction mode
     * @param newDirection  New pass direction mode
     */
    public void setPassDirection(PassDirection newDirection) 
    {
        direction = newDirection;
    }

    /**
    * Returns the current pass direction of the robot.
    */
    public PassDirection getPassDirection() 
    {
        return direction;
    }

    /**
     * True if the direction is left, false otherwise
     */
    public boolean onLeftSide()
    {
      if (direction == PassDirection.Left) return true;
      if (direction == PassDirection.Right) return false;
      return (Util.onLeftSide(drivetrain));
    }

    /**
     * Returns Subsystem status for the given subsys index
     */
    public SubsystemStatus getStatus(int subsystem)
    {
        return modeChoosers.get(subsystem).getSelected();
    }

    /**
     * Returns subsystem status and handles sim running
     */
    private static String subsystemName(int subsystem)
    {
        return switch(subsystem) {
            case 0 -> "Intake";
            case 1 -> "IntakeExtension";
            case 2 -> "Climb";
            case 3 -> "Hood";
            case 4 -> "Hopper";
            case 5 -> "Indexer";
            case 6 -> "Shooter";
            case 7 -> "ShooterIndexer";
            default -> "";
        };
    }
  
        
    /**
     * Constructs the RobotContainer 
     * and initializes subsystem mode choosers
     */    
    public RobotContainer() 
    {
        for (int i = 0; i < INDEXES; i++)
        {
            SendableChooser<SubsystemStatus> chooser = new SendableChooser<>();

            if (Robot.isSimulation())
            {
                chooser.setDefaultOption("Sim", SubsystemStatus.Simulated);
                chooser.addOption("Off", SubsystemStatus.Disabled);
            }
            else
            {
                chooser.setDefaultOption("On", SubsystemStatus.Enabled);
                chooser.addOption("Off", SubsystemStatus.Disabled);
                chooser.addOption("Sim", SubsystemStatus.Simulated);
            }
            modeChoosers.add(chooser);
            SmartDashboard.putData(subsystemName(i), chooser);
        }
    }

    /**
     * Post-construction initialization
     * Builds commands, registers PathPlanner NamedCommands, configures default subsystem commands, 
     * selects control binding layout and publishes choosers to SmartDashBoard
     */
    public void init()
    {
        stow = ()->
            new IndexerStartDefaultSpeed()
            .alongWith(new StartDefaultIntake())
            .alongWith(new ShooterTargetSpeed(()->Constants.DEFAULT_FLYWHEEL_VELOCITY.in(MetersPerSecond)))
            .alongWith(new ShooterIndexerStartDefaultSpeed())
            .andThen(()->isShooting = false)
            .andThen(new AimToAngle(()->75.0))
            .withName("Stow"); // must stay a supplier

        rev = () -> new ShooterIndexerStartDefaultSpeed()
                .andThen(new ShooterTargetSpeed(
                    isRevShoot ? ()->Util.calculateShootVelocity(drivetrain) : ()->Constants.MID_PASS_VELOCITY.in(MetersPerSecond)
                    ))
                .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot()))
                .andThen(Commands.runOnce(()->driver.setRumble(RumbleType.kBothRumble, 1.0)))
            .withName("Rev");

        testCommandChooser.setDefaultOption("None", Commands.none());
        testCommandChooser.addOption("Climb/ClimbUp", new ClimbUp());
        testCommandChooser.addOption("Climb/ClimbDown", new ClimbDown());
        testCommandChooser.addOption("Climb/SpoolUntilStall", new Spool());
        testCommandChooser.addOption("Climb/Unspool", new Unspool());
        testCommandChooser.addOption("Hood/AimToAngle[60°]", new AimToAngle(()->60.0));
        testCommandChooser.addOption("Hood/AimToAngle[70°]", new AimToAngle(()->70.0));
        testCommandChooser.addOption("Hood/AimToAngle[75°]", new AimToAngle(()->75.0));
        testCommandChooser.addOption("Hood/ZeroHood", new ZeroHood());
        testCommandChooser.addOption("Indexer/IndexerStartDefaultSpeed", new IndexerStartDefaultSpeed());
        testCommandChooser.addOption("Indexer/IndexerStartFullSpeed", new IndexerStartFullSpeed());
        testCommandChooser.addOption("Intake/StartDefaultIntake", new StartDefaultIntake());
        testCommandChooser.addOption("Intake/StartRunIntake", new StartRunIntake());
        testCommandChooser.addOption("Intake/StartEjectIntake", new StartEjectIntake());
        testCommandChooser.addOption("Intake/AgitateIntake", new AgitateIntake());
        testCommandChooser.addOption("IntakeExtension/ExtendIntake", new ExtendIntake());
        testCommandChooser.addOption("IntakeExtension/RetractIntake", new RetractIntake());
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[10]", new ShooterTargetSpeed(()->10.0));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + Constants.HARDCODE_VELOCITY + "]", new ShooterTargetSpeed(()->Constants.HARDCODE_VELOCITY.in(MetersPerSecond)));
        testCommandChooser.addOption("Shooter/ShooterTargetSpeed[" + 0 + "]", Shooter.getInstance().runOnce(()->Shooter.getInstance().setVoltage(Volts.of(0.0))));
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerStartDefaultSpeed", new ShooterIndexerStartDefaultSpeed());
        testCommandChooser.addOption("ShooterIndexer/ShooterIndexerStartFullSpeed", new ShooterIndexerStartFullSpeed());
        testCommandChooser.addOption("ZeroDrivetrain", 
                drivetrain.runOnce(() -> {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                {
                    drivetrain.seedFieldCentric(Rotation2d.k180deg);
                }
                else
                {
                    drivetrain.seedFieldCentric();
                }
            })
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));

        SmartDashboard.putData("Test a Command", testCommandChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());

        NamedCommands.registerCommand("ExtendIntake", Commands.runEnd(
            ()->IntakeExtension.getInstance().setVoltage(Constants.IntakeExtension.EXTENDING_VOLTAGE),
            ()->IntakeExtension.getInstance().setVoltage(Constants.IntakeExtension.HOLDING_EXTEND_VOLTAGE)));
        NamedCommands.registerCommand("RetractIntake", new RetractIntake().alongWith(new StartRunIntake())
            .andThen(new StartDefaultIntake()));
        NamedCommands.registerCommand("StartRunIntake", new StartRunIntake());
        NamedCommands.registerCommand("StartDefaultIntake", new StartDefaultIntake());
        NamedCommands.registerCommand("EjectIntake (with timeout)", 
            new StartEjectIntake()
            .andThen(new WaitCommand(1.0))
            .andThen(new StartDefaultIntake()));
        
        NamedCommands.registerCommand("HardShoot", 
            new ShooterTargetSpeed(()->Constants.HARDCODE_VELOCITY.in(MetersPerSecond))
            .andThen(new AimToAngle(()->Constants.HARDCODE_HOOD_PITCH.in(Degrees)))
            .andThen(new WaitUntilCommand(()->Hood.getInstance().readyToShoot() && Shooter.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()));



        NamedCommands.registerCommand("RevShoot",
            new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)));
        NamedCommands.registerCommand("LinearAgitateIntake",
            new StartRunIntake()
            .andThen(new RetractIntake().withTimeout(2.0))
            .andThen(new ExtendIntake().withTimeout(1.0))
            .andThen(new RetractIntake().withTimeout(2.0))
            .andThen(new ExtendIntake().withTimeout(1.0))
            .andThen(new RetractIntake().withTimeout(2.0)));

        NamedCommands.registerCommand("Shoot", 
            new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees))
            .andThen(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))
            .andThen(new WaitUntilCommand(()->Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()) // load to shoot
            .andThen(Commands.run(()->{}))
        .withName("Shoot"));

        NamedCommands.registerCommand("Stow", stow.get());
        
        autonChooser = AutoBuilder.buildAutoChooser();
        
        SmartDashboard.putData("Auton Chooser", autonChooser);

        // ----------------------- DEFAULT BINDINGS HERE -------------------------

        Hood.getInstance().setDefaultCommand(new HoodManual());
        CommandScheduler.getInstance().schedule(stow.get());

        // -------------------- CHANGE BINDING SETTINGS HERE ---------------------
        
        boolean useDebuggingBindings = false; // mainly for sysid or debugging
        
        if (useDebuggingBindings) configureDebugBindings();
        else
        {
            configureDriverBindings();
        }
        drivetrain.registerTelemetry(Telemetry.getInstance()::telemeterize);
    }


    /**
     * Configures debugging bindings
     * Used during development, not for match play
     */
    private void configureDebugBindings()
    {
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> 
                        drive.withVelocityX(-driver.getLeftY() * MaxSpeed.in(MetersPerSecond)) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed.in(MetersPerSecond) * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate.in(RadiansPerSecond) * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                    ).withName("SwerveManual"));
    }
    
    /**
     * Configures all driver controller bindings for driving, shooting, passing, and intake control.
     */
    private void configureDriverBindings() 
    {
        driver.b().whileTrue(
                new AimToAngle(()->Constants.HARD_PASS_ANGLE.in(Degrees))
                .andThen(new ShooterTargetSpeed(()->Constants.HARD_PASS_VELOCITY.in(MetersPerSecond)))
                .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
                .andThen(new IndexerStartFullSpeed())
                .andThen(new ShooterIndexerStartFullSpeed())
            .withName("HardPass"));

        driver.b().onFalse(stow.get());
        
        driver.y().whileTrue(
            new AimToAngle(()->Constants.SOFT_PASS_ANGLE.in(Degrees))
            .andThen(new ShooterTargetSpeed(()->Constants.SOFT_PASS_VELOCITY.in(MetersPerSecond)))
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(new IndexerStartFullSpeed())
            .andThen(new ShooterIndexerStartFullSpeed())
            .withName("SoftPass"));

        driver.y().onFalse(stow.get());
        
        driver.a().whileTrue(
        // lucas wanted to remove the auto-aligning (4/3/26, at contra costa) 
        // new RotateToAngle(drivetrain,
        //     () -> onLeftSide() ? Constants.PASS_LEFT_TARGET_POSITION.toTranslation2d()
        //                        : Constants.PASS_RIGHT_TARGET_POSITION.toTranslation2d(), true)
                new AimToAngle(()->Constants.MID_PASS_ANGLE.in(Degrees))
                .andThen(new ShooterTargetSpeed(()->Constants.MID_PASS_VELOCITY.in(MetersPerSecond)))
                .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
                .andThen(new IndexerStartFullSpeed())
                .andThen(new ShooterIndexerStartFullSpeed())
            .withName("MidPass"));

        driver.a().onFalse(stow.get());
        
        driver.x().onTrue(
            new ShooterTargetSpeed(()->Constants.HARDCODE_VELOCITY.in(MetersPerSecond))
            .andThen(new AimToAngle(()->Constants.HARDCODE_HOOD_PITCH.in(Degrees)))
            .andThen(new WaitUntilCommand(()->Hood.getInstance().readyToShoot() && Shooter.getInstance().readyToShoot()))
            .andThen(()->isShooting = true)
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed())
            .withName("HardShoot"));

        driver.x().onFalse(stow.get());


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                    drive.withVelocityX(/*accelerationLimiter.calculate(*/-driver.getLeftY() * MaxSpeed.in(MetersPerSecond) * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)/*)*/) // Drive forward with negative Y (forward)
                    .withVelocityY(/*accelerationLimiter.calculate(*/-driver.getLeftX() * MaxSpeed.in(MetersPerSecond) * (isSlow ? Constants.TRANSLATION_SLOW_MULTIPLIER : 1.0)/*)*/) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate.in(RadiansPerSecond) * (isSlow ? Constants.ROTATION_SLOW_MULTIPLIER : 1.0)) // Drive counterclockwise with negative X (left)
                ).withName("SwerveManual").onlyIf(()-> !isShooting));

        //changed from pass to eject
        /*
        // tested
        driver.leftTrigger().whileTrue(
            new RotateToAngle(drivetrain, ()->{
                Pose2d drivetrainPose = drivetrain.getState().Pose;
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                {
                    if (drivetrainPose.getY() < Constants.Simulation.FIELD_HEIGHT.in(Meters))
                    {
                        return Constants.PASS_LEFT_TARGET_POSITION;
                    }
                    else
                    {
                        return Constants.PASS_RIGHT_TARGET_POSITION;
                    }
                }
                else
                {
                    if (drivetrainPose.getY() < Constants.Simulation.FIELD_HEIGHT.in(Meters))
                    {
                        return FlippingUtil.flipFieldPosition(Constants.PASS_RIGHT_TARGET_POSITION);
                    }
                    else
                    {
                        return FlippingUtil.flipFieldPosition(Constants.PASS_LEFT_TARGET_POSITION);
                    }
                }
            }, false)
            .alongWith(new AimToAngle(()->Constants.MID_PASS_ANGLE.in(Degrees)))
            .alongWith(new ShooterTargetSpeed(()->Constants.MID_PASS_VELOCITY.in(MetersPerSecond)))
            .andThen(new WaitUntilCommand(()->Hood.getInstance().readyToShoot() && Shooter.getInstance().readyToShoot()))
            .andThen(new ShooterIndexerStartFullSpeed())
            .withName("Pass")
        );

        driver.leftTrigger().onFalse(stow.get().andThen(Commands.runOnce(()->{
            Command currentDrivetrainCommand = drivetrain.getCurrentCommand();
            if (currentDrivetrainCommand instanceof RotateToAngle) CommandScheduler.getInstance().cancel(currentDrivetrainCommand);
        })));
        */

        driver.leftTrigger().whileTrue(new StartEjectIntake()
            .andThen(new IndexerStartEjectSpeed())
            .andThen(new ShooterIndexerStartEjectSpeed()).withName("Eject"));
        driver.leftTrigger().onFalse(new StartRunIntake()
            .andThen(new IndexerStartDefaultSpeed())
            .andThen(new ShooterIndexerStartDefaultSpeed()));

        driver.rightTrigger().whileTrue(
            Constants.DATA_COLLECTION_MODE ?
            new AimToAngle(()->Telemetry.getInstance().getHoodAngle().in(Degrees))
            .andThen(new ShooterTargetSpeed(()->Telemetry.getInstance().getShooterSpeed().in(MetersPerSecond)))
            .andThen(new WaitCommand(2.0))
            .andThen(new ShooterIndexerStartFullSpeed())
            .andThen(new IndexerStartFullSpeed()).withName("DCShoot") : 
            
            new RotateToAngle(drivetrain, ()->{
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
                    return FlippingUtil.flipFieldPosition(Constants.AlignConstants.HUB);
                return Constants.AlignConstants.HUB;
            }, false)
            .alongWith(new AimToAngle(()->Util.calculateShootPitch(drivetrain).in(Degrees)))
            .alongWith(new ShooterTargetSpeed(()->Util.calculateShootVelocity(drivetrain)))
            .andThen(new WaitUntilCommand(() -> Shooter.getInstance().readyToShoot() && Hood.getInstance().readyToShoot()))
            .andThen(()->isShooting=true)
            .andThen(new IndexerStartFullSpeed())
            .andThen(new ShooterIndexerStartFullSpeed())
            .withName("Shoot")
            );

        driver.rightTrigger().onFalse(stow.get().andThen(Commands.runOnce(()->{
            Command currentDrivetrainCommand = drivetrain.getCurrentCommand();
            if (currentDrivetrainCommand instanceof RotateToAngle) CommandScheduler.getInstance().cancel(currentDrivetrainCommand);
        })));

        driver.leftBumper().onTrue(stow.get());

        //changed from retract/extand hopper and intake
        driver.rightBumper().onTrue(
            new StartRunIntake()
            .andThen(
                new RetractIntake()
                .alongWith(
                    new WaitUntilCommand(Intake.getInstance()::isStalling)
                    .andThen(new StartDefaultIntake())
                    .andThen(Commands.runOnce(()->intakeTriggered = false)))
            ).withName("Retracting Intake"));

        driver.rightBumper().onFalse(
            new StartRunIntake()
            .andThen(Commands.runOnce(()->intakeTriggered = true))
            .andThen(new ExtendIntake()).withName("Extending Intake")
            );

        //removed rev pass
        /*
        driver.button(7).onTrue( // home button/left paddle
                Commands.runOnce(()->isRevShoot = false)
                .andThen(rev.get())
            .withName("RevPass"));
        */

        driver.button(8).onTrue( // menu button/right paddle
                Commands.runOnce(()->isRevShoot = true)
                .andThen(rev.get())
            .withName("RevShoot"));

        driver.povUp().onTrue(
                drivetrain.runOnce(() -> {
                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red)
                {
                    drivetrain.seedFieldCentric(Rotation2d.k180deg);
                }
                else
                {
                    drivetrain.seedFieldCentric();
                }
            })
                .andThen(drivetrain.runOnce(() -> drivetrain.resetPose(
                            (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? 
                            FlippingUtil.flipFieldPose(Constants.ZEROING_POSE) : Constants.ZEROING_POSE)))
                .withName("ZeroDrivetrain"));

        driver.povDown().whileTrue(new ZeroHood().withName("ZeroHood"));

        driver.povRight().onTrue(
            Intake.getInstance().runOnce(()->Intake.getInstance().setVelocity(Constants.Intake.REDUCED_INTAKE_VELOCITY))
            .andThen(Commands.runOnce(()->intakeTriggered = true))
            .andThen(new RetractIntake())
            .andThen(new StartDefaultIntake())
            .andThen(Commands.runOnce(()->intakeTriggered = false))
            .andThen(()->intakeExtended = false)
            .withName("HardRetract"));

        driver.povLeft().onTrue(
            new ExtendIntake()
            .andThen(Commands.runOnce(()->
            {
                intakeExtended = true;
            })
            .andThen(new StartRunIntake())
            .andThen(Commands.runOnce(()->intakeTriggered = true
            )).withName("HardExtend")));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Drivetrain Set Idle"));
                
    }

    /**
     * Returns the current tower auto alignment side
     */
    public AlignDirection getAlignDirection ()
    {
        return alignDirection;
    }

    /**
     * Sets the tower alignment side
     * Called by the align-direction commands bound to driver buttons.
     */
    public void setAlignDirection (AlignDirection direction)
    {
        this.alignDirection = direction;
    }

    /**
     * Returns the autonomous command selected.
     * Called at the start of autonomous.
     */
    public Command getAutonomousCommand()
    {
        return autonChooser.getSelected();
    }
}

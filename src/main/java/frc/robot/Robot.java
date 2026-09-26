// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.ByteBuffer;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.hood.ZeroHood;
import frc.robot.simulation.SimulationState;
import frc.robot.util.Util;

/**
 * Main robot entry point that manages the robot mode lifecycle and scheduling
 */
public class Robot extends TimedRobot 
{
   /**
    * The way that this code works is that, during autonomous mode, it first schedules
    * zeroHood and schedules the autonomous command once the zeroHood command has ended
    * (ie., once the hood stalls). Before this we simply did
    * CommandScheduler.getInstance().schedule(new ZeroHood().andThen(autonomousCommand)),
    * but this crashes the code if autonomous mode is entered multiple times since you
    * cannot compose a command multiple times. Normally the way around this would be to
    * create a new instance of the autonomous command each time but pathplanner doesn't
    * permit this (AutoChooser) and there is no way to copy commands (!). After consideration,
    * this is the best way to resolve the issue to prevent the code from crashing during
    * a match. Other solutions could be to add ZeroHood to the beginning of all of the autons
    * or to always remember to restart robot code before matches and when testing autons.
    */
   private Command zeroHoodCommand;
   private boolean hasStartedAutonomousCommand = false;
   private Command autonomousCommand;
   public RobotContainer robotContainer;
   public static Robot instance;


   public Robot() 
   {
      instance = this;
      Telemetry.getInstance();
      robotContainer = new RobotContainer();
      


      // automatically saves log data for telemetry, driver station controls, and joystick presses
      if ((isReal() && Constants.LOG_REAL) || Constants.LOG_SIMULATION)
      {
         DataLogManager.start();
         DriverStation.startDataLog(DataLogManager.getLog());
      }
   }
   
   /**
    * Called when the robot starts
    * Initializes RobotContainer bindings and constructs a LimelightSimulation if in simulation
    */
   @Override
   public void robotInit() 
   {
      robotContainer.init();
      Util.init();

      zeroHoodCommand = new ZeroHood(); // this must be here since Hoood doesn't exist when the class is created or the constructor is called


      LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.CAMERA_1_NAME, 
      Constants.Vision.ROBOT_TO_CAMERA_1.getX(), Constants.Vision.ROBOT_TO_CAMERA_1.getY(), Constants.Vision.ROBOT_TO_CAMERA_1.getZ(),
      Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_1.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_1.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_1.getRotation().getZ()));
   

      LimelightHelpers.setCameraPose_RobotSpace(Constants.Vision.CAMERA_2_NAME, 
      Constants.Vision.ROBOT_TO_CAMERA_2.getX(), Constants.Vision.ROBOT_TO_CAMERA_2.getY(), Constants.Vision.ROBOT_TO_CAMERA_2.getZ(),
      Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_2.getRotation().getX()), Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_2.getRotation().getY()), Units.radiansToDegrees(Constants.Vision.ROBOT_TO_CAMERA_2.getRotation().getZ()));
   }
      
   /**
    * Runs every 20ms regardless of robot mode
    */
   @Override
   public void robotPeriodic() 
   {
      CommandScheduler.getInstance().run();

      CommandScheduler.getInstance().schedule(robotContainer.testCommandChooser.getSelected());

      Telemetry.getInstance().update();
   }

   /**
    * Called when entering disabled mode
    * Stops SignalLogger if on real hardware
    */
   @Override
   public void disabledInit() 
   {
      robotContainer.driver.setRumble(RumbleType.kBothRumble, 0.0);

      if(isReal() && Constants.LOG_CTRE)
      {
         SignalLogger.stop();
      }
   }
   /**
    * Called periodically while disabled
    * No actions are required
    */
   @Override
   public void disabledPeriodic() {}

   /**
    * Called when exiting disabled mode
    * Re-starts SignalLogger if on real hardware
    */
   @Override
   public void disabledExit() 
   {
      if (isReal() && Constants.LOG_CTRE)
      {
         SignalLogger.start();
      }
   }
   /**
    * Called when entering autonomous mode
    * Grabs the selected autonomous command and schedules it if it exists
    */
   @Override
   public void autonomousInit() 
   {
      autonomousCommand = robotContainer.getAutonomousCommand();

      CommandScheduler.getInstance().schedule(zeroHoodCommand);
      hasStartedAutonomousCommand = false;
   }
   /**
    * Called periodically during autonomous mode
    */
   @Override
   public void autonomousPeriodic() 
   {
      if (!zeroHoodCommand.isScheduled() && !hasStartedAutonomousCommand)
      {
         hasStartedAutonomousCommand = true;
         if (autonomousCommand != null)
         {
            System.out.println("Scheduling autonomous command");
            CommandScheduler.getInstance().schedule(autonomousCommand);
         }
      }
   }

   /**
    * Called when exiting autonomous mode
    */
   @Override
   public void autonomousExit() 
   {
      autonomousCommand = null;
   }

   /**
    * Called when entering teleoperated mode
    * If an autonomous command is still running, it is canceled
    */
   @Override
   public void teleopInit() 
   {

      CommandScheduler.getInstance().schedule(new ZeroHood());
      if (autonomousCommand != null) 
      {
         autonomousCommand.cancel();
      }
   }

   /**
    * Called periodically during teleoperated mode
    * No logic is required
    */
   @Override
   public void teleopPeriodic() 
   {
   }

   /**
    * Called when exiting teleoperated mode
    * No actions are required
    */
   @Override
   public void teleopExit() {}

   /**
    * Called when entering test mode
    * All running commands are canceled
    */
   @Override
   public void testInit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }

   /**
    * Called periodically during test mode
    * No logic is required
    */
   @Override
   public void testPeriodic() {}

   /**
    * Called when exiting test mode
    * All running commands are canceled
    */
   @Override
   public void testExit() 
   {
      CommandScheduler.getInstance().cancelAll();
   }

   /**
    * Called when entering simulation mode
    * No simulation initialization is required here
    */
   @Override
   public void simulationInit() 
   {
   }

   /**
    * Called periodically during simulation mode
    * Updates the simulation state each loop
    */
   @Override
   public void simulationPeriodic()
   {
      SimulationState.getInstance().update();
   }
}

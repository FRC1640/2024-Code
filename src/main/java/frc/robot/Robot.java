// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.periodic.PeriodicScheduler;
import frc.robot.subsystems.drive.DriveWeightCommand;
import frc.robot.util.dashboard.PIDUpdate;

public class Robot extends LoggedRobot {
    public static enum Mode {
        REAL, SIM, REPLAY
    };

    public static enum TestMode {
        NONE, SYSID, PID
    };

    public static boolean inTeleop;

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("RobotMode", getMode().toString());
        // Logger.recordMetadata("MACAddress", getMACAddress());
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        System.out.println(getMode().toString());
        // Set up data receivers & replay source
        switch (getMode()) {
            // Running on a real robot, log to a USB stick
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            // Running a physics simulator, log to local folder
            case SIM:
                Logger.addDataReceiver(new WPILOGWriter("logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            // Replaying a log, set up replay source
            case REPLAY:
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.getInstance().disableDeterministicTimestamps()

        // Start AdvantageKit Logger
        Logger.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {

        
        // Runs the PeriodicScheduler. This is responsible for running periodic()
        // in all PeriodicBase instances.
        PeriodicScheduler.getInstance().run();

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        Logger.recordOutput("Memory", Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory());

        m_robotContainer.get3dDistance(()->m_robotContainer.getSpeakerPos());
    }

    @Override
    public void disabledInit() {
        DriveWeightCommand.removeAllWeights();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        inTeleop = true;
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        switch (DashboardInit.getTestMode()) {
            case NONE:
                System.out.println("Nothing is happening!");
                break;
            case SYSID:
                if (DashboardInit.getSelectedSysid() != null){
                    System.out.println("Running sysid on: " + DashboardInit.getSelectedSysid().getName());
                    DashboardInit.getSelectedSysid().schedule();
                    CommandScheduler.getInstance().getActiveButtonLoop().clear();
                    m_robotContainer.removeAllDefaultCommands();
                }
                break;
            case PID:
                System.out.println("PID MODE");
                m_robotContainer.removeAllDefaultCommands();
                m_robotContainer.pidTriggers();
                break;
        }
    }

    @Override
    public void testPeriodic() {
        switch (DashboardInit.getTestMode()) {
            case NONE:
                break;
            case SYSID:
                break;
            case PID:
                PIDUpdate.periodic();
                break;
        }
    }

    @Override
    public void testExit() {
    }

    public static boolean isReplay() {
        String replay = System.getProperty("REPLAY");
        return replay != null && replay.toLowerCase().equals("true");
    }

    public static Mode getMode() {
        if (isReal()) {
            return Mode.REAL;
        }

        if (isReplay()) {
            return Mode.REPLAY;
        }

        return Mode.SIM;
    }

    public static String getMACAddress() {
        try {
            InetAddress localHost = InetAddress.getLocalHost();
            NetworkInterface ni = NetworkInterface.getByInetAddress(localHost);
            byte[] hardwareAddress = ni.getHardwareAddress();
            String[] hexadecimal = new String[hardwareAddress.length];
            for (int i = 0; i < hardwareAddress.length; i++) {
                hexadecimal[i] = String.format("%02X", hardwareAddress[i]);
            }
            return String.join("-", hexadecimal);
        } catch (UnknownHostException | SocketException e) {
            e.printStackTrace();
            return "Not found";
        }
    }
}
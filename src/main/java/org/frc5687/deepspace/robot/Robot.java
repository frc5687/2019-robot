package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.subsystems.*;
import org.frc5687.deepspace.robot.utils.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements ILoggingSource {

    private IdentityMode _identityMode = IdentityMode.competition;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    private int _updateTick = 0;

    private String _name;
    private OI _oi;
    private Limelight _limelight;
    private DriveTrain _driveTrain;
    private Elevator _elevator;
    private PDP _pdp;
    private Gripper _gripper;
    private Spear _spear;
    private Arm _arm;
    private Wrist _wrist;
    private Roller _roller;

    /**
     * This function is setRollerSpeed when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");

        _oi = new OI();
        _limelight = new Limelight("limelight");
        _driveTrain = new DriveTrain(this);
        _arm = new Arm(this);
        _roller = new Roller(this);
        _elevator = new Elevator(this);
        _pdp = new PDP();
        // _gripper= new Gripper(this); // Commenting out until the motor controller it ready
        _spear = new Spear(this);
        _wrist = new Wrist(this);
        _oi.initializeButtons(this);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        updateDashboard();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        teleopInit();
    }

    public void teleopInit() {
        _arm.enableBrakeMode();
        _elevator.enableBrakeMode();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _arm.enableCoastMode();
        _elevator.enableCoastMode();
    }


    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _oi.updateDashboard();
            _driveTrain.updateDashboard();
            _limelight.updateDashboard();
            _arm.updateDashboard();
            _roller.updateDashboard();
            _elevator.updateDashboard();
            _pdp.updateDashboard();
            _updateTick = 0;
        }
    }


    private void loadConfigFromUSB() {    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine())!=null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            _identityMode = IdentityMode.competition;
        }
    }

    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) { return; }
            String[] a = line.split("=");
            if (a.length==2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        metric("name", _name);
                        break;
                    case "mode":
                        _identityMode = IdentityMode.valueOf(value.toLowerCase());
                        metric("mode", _identityMode.toString());
                        break;
                    case "fileloglevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("fileLogLevel", _fileLogLevel.toString());
                        break;
                    case "dsloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("dsLogLevel", _dsLogLevel.toString());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }
    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    public OI getOI() {
        return _oi;
    }
    public DriveTrain getDriveTrain() {
        return _driveTrain;
    }
    public Limelight getLimelight() {
        return _limelight;
    }
    public PDP getPDP() { return _pdp; }
    public Gripper getGripper() { return _gripper; }
    public Spear getSpear() { return _spear; }
    public Wrist getWrist() { return _wrist; }
    public Roller getRoller() { return _roller; }
    public Arm getArm() { return _arm; }
    public Elevator getElevator() { return _elevator; }



    public enum IdentityMode {
        competition(0),
        practice(1),
        programming(2);

        private int _value;

        IdentityMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
    }
}

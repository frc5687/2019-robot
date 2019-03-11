package org.frc5687.deepspace.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.deepspace.robot.commands.KillAll;
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

    public static IdentityMode identityMode = IdentityMode.competition;
    private Configuration _configuration;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    private int _updateTick = 0;

    private String _name;
    private OI _oi;
    private AHRS _imu;
    private Limelight _limelight;
    private DriveTrain _driveTrain;
    private Elevator _elevator;
    private PDP _pdp;
    private Arm _arm;
    private Shifter _shifter;
    private Lights _lights;
    private Stilt _stilt;
    private CargoIntake _cargoIntake;
    private HatchIntake _hatchIntake;

    /**
     * This function is setRollerSpeed when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        metric("Branch", Version.BRANCH);
        info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        info("Robot " + _name + " running in " + identityMode.toString() + " mode");

        // Periodically flushes metrics (might be good to configure enable/disable via USB config file)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);

        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        _imu.zeroYaw();

        // then proxies...
        _lights = new Lights(this);
        _limelight = new Limelight("limelight");
        _pdp = new PDP();

        // Then subsystems....
        _shifter = new Shifter(this);
        _driveTrain = new DriveTrain(this);
        _arm = new Arm(this);
        _elevator = new Elevator(this);
        _stilt = new Stilt(this);
        _cargoIntake = new CargoIntake(this);
        _hatchIntake = new HatchIntake(this);

        // Must initialize buttons AFTER subsystems are allocated...
        _oi.initializeButtons(this);

        // Initialize the other stuff
//        _limelight.disableLEDs();
        _limelight.setStreamingMode(Limelight.StreamMode.PIP_SECONDARY);
        setConfiguration(Configuration.starting);
        _arm.resetEncoders();
        _arm.enableBrakeMode();
        _elevator.enableBrakeMode();
        _stilt.enableBrakeMode();

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
        _oi.poll();
        update();
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
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        ourPeriodic();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        ourPeriodic();
    }

    private void ourPeriodic() {
        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();

        int operatorPOV = _oi.getOperatorPOV();
        int driverPOV = _oi.getDriverPOV();


        if (driverPOV != 0 || operatorPOV != 0) {
            new KillAll(this).start();
        }

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
        //_limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _arm.enableCoastMode();
        _elevator.enableCoastMode();
        _stilt.enableCoastMode();
        MetricTracker.flushAll();
    }


    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _oi.updateDashboard();
            _driveTrain.updateDashboard();
            _limelight.updateDashboard();
            _arm.updateDashboard();
            _elevator.updateDashboard();
            _pdp.updateDashboard();
            _shifter.updateDashboard();
            _lights.updateDashboard();
            _stilt.updateDashboard();
            _cargoIntake.updateDashboard();
            _hatchIntake.updateDashboard();
            metric("imu/yaw", _imu.getYaw());
            metric("imu/pitch", _imu.getPitch());
            metric("imu/roll", _imu.getRoll());
            metric("imu/version", _imu.getFirmwareVersion());
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
            identityMode = IdentityMode.competition;
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
                        identityMode = IdentityMode.valueOf(value.toLowerCase());
                        metric("mode", identityMode.toString());
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
    public void setConfiguration(Configuration configuration) {
        _configuration = configuration;
    }
    public Configuration getConfiguration() {
        return _configuration;
    }

    private boolean _wereLEDsOn = false;
    private boolean _trackingRetrieveHatch = false;
    private boolean _trackingScoreHatch = false;

    private void update() {
        switch (_configuration) {
            case starting:
                if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Red) {
                    _lights.setColor(Constants.Lights.PULSING_RED, 0);
                } else if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Blue) {
                    _lights.setColor(Constants.Lights.PULSING_BLUE, 0);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_YELLOW, 0);
                }
                break;
            case hatch:
                // In hatch mode,
                //  - intake pointed
                //    - limelight on - started while no hatch
                //    - target sighted, started while no hatch
                //    - target centered, started while no hatch
                //    - no hatch - pulsing purple
                //    - hatch detected - solid purple
                //  - intake gripped -
                //    - limelight on - started while had hatch
                //    - target sighted, started while had hatch
                //    - target centered, started while had hatch
                //    - has hatch - solid purple
                //    - no hatch - pale puple
                if (_hatchIntake.isPointed()) {
                    if (_limelight.areLEDsOn()) {
                        if (!_wereLEDsOn) {
                            _trackingRetrieveHatch = true;
                        }
                    }
                    if (_hatchIntake.isHatchDetected()) {
                        _lights.setColor(Constants.Lights.SOLID_PURPLE, 0);
                    } else if (_trackingRetrieveHatch) {
                        if (_limelight.isTargetCentered()) {
                            _lights.setColor(Constants.Lights.SOLID_GREEN, 0);
                        } else if (_limelight.isTargetSighted()) {
                            _lights.setColor(Constants.Lights.PULSING_GREEN, 0);
                        } else {
                            _lights.setColor(Constants.Lights.SOLID_GREEN, 1);
                        }
                    } else {
                        _lights.setColor(Constants.Lights.PULSING_PURPLE, 0);
                    }
                } else {
                    if (_limelight.areLEDsOn()) {
                        if (!_wereLEDsOn) {
                            _trackingScoreHatch = true;
                        }
                    }
                    if (_trackingScoreHatch) {
                        if (_limelight.isTargetCentered()) {
                            _lights.setColor(Constants.Lights.SOLID_GREEN, 0);
                        } else if (_limelight.isTargetSighted()) {
                            _lights.setColor(Constants.Lights.PULSING_GREEN, 0);
                        } else {
                            _lights.setColor(Constants.Lights.SOLID_GREEN, 1);
                        }
                    } else if (_hatchIntake.isHatchDetected()) {
                        _lights.setColor(Constants.Lights.SOLID_PURPLE, 0);
                    } else {
                        _lights.setColor(Constants.Lights.PULSING_PURPLE, 1);
                    }
                }

                break;
            case cargo:
                // Intake running?
                //   Has cargo? SOLID ORANGE
                //   Targeting?
                //   Detected?
                //   Centered?
                //   No?  PULSING ORANGE
                // Intake not running
                //   Has cargo? SOLID ORANGE
                //   No cargo? PALE

                if (false) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 0);
                } else if (_cargoIntake.isBallDetected()) {
                    _lights.setColor(Constants.Lights.SOLID_ORANGE, 4);
                } else {
                    _lights.setColor(Constants.Lights.PULSING_ORANGE, 2);
                }
                break;
            case climbing:
                _lights.setColor(Constants.Lights.PULSING_YELLOW, 0);
                break;
            case parked:
                _lights.setColor(Constants.Lights.CONFETTI, 0);
                break;
        }
        _wereLEDsOn = _limelight.areLEDsOn();
        if (!_wereLEDsOn) {
            _trackingScoreHatch = false;
            _trackingRetrieveHatch = false;
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
    public AHRS getIMU() { return _imu; }
    public DriveTrain getDriveTrain() { return _driveTrain; }
    public Limelight getLimelight() {
        return _limelight;
    }
    public PDP getPDP() { return _pdp; }
    public Arm getArm() { return _arm; }
    public Elevator getElevator() { return _elevator; }
    public Shifter getShifter() { return _shifter; }
    public Lights getLights() { return _lights; }
    public Stilt getStilt() { return _stilt; }
    public CargoIntake getCargoIntake() { return _cargoIntake;}
    public HatchIntake getHatchIntake() { return _hatchIntake;}

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

    public enum Configuration {
        starting(0),
        hatch(1),
        cargo(2),
        climbing(3),
        parked(4);

        private int _value;

        Configuration(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public IdentityMode getIdentityMode() {
        return identityMode;
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

// ***********************************************************************
// SwerveCore
// ***********************************************************************
// Extends the OpMode class to provide a single hardware access point for our robot.
//
// It also gives a common way to do things like reading and saving autonomous settings
//
// All operation control for the swerve robot starts here.
//
// We have included all of the motor and servo definitions, so that they do not have to be
// kept up in all of the individual files.
//
// We also included the functions for moving the various key motors and servos or any other
// basic robot operations. It simplifies the code in our other program files.


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
//
// Not used in our core class extension
// ***********************************************************************
@TeleOp(name="Swerve: 99-core", group="Swerve")
@Disabled

// ***********************************************************************
// SwerveCore
// ***********************************************************************
// Class definitions

public class SwerveCore extends OpMode {
    //
    // Hardware mapping of devices
    //

    //
    // Swerve drive is 4 wheels turned by 4 servos
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightRear;
    DcMotor motorLeftRear;
    Servo   servoRightFront;
    Servo   servoLeftFront;
    Servo   servoRightRear;
    Servo   servoLeftRear;

    SwerveWheel swerveRightFront;
    SwerveWheel swerveLeftFront;
    SwerveWheel swerveRightRear;
    SwerveWheel swerveLeftRear;
    SwerveDrive ourSwerve;

    // servos for game elements
    Servo gameMarkDrop;
    Servo foundationLeft;
    Servo foundationRight;

    // *** Sensors ***
    TouchSensor foundationPress;
    DistanceSensor heightR;
    DistanceSensor heightL;
    // battery value in the hub
    VoltageSensor   batteryVoltSensor;

    // inertial management in the hub
    BNO055IMU ourIMU;
    // IMU calibration file
    String imuCalibration = "RevIMUCalibration.json";


    //
    // Values for autonomous activities
    //
    Boolean crater;
    // Delay at start
    int autoDelay;
    static final int delayMAX = 7000;


    //
    // Controller handling values
    //

    // When reading the controllers, ignore small bumps, stick and trigger deadzone
    public static double minJoystickMove = 0.2;
    public static double minTriggerMove = 0.2;


    // Run time data
    private String startDate;
    private ElapsedTime fullTime = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    // Number output format
    public DecimalFormat swerveNumberFormat;


    // Level of debug data to show on driver station
    int     debugLevel = 499;


    // ***********************************************************************
    // SwerveCore
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveCore() {
        // Initialize base classes.
        // All via self-construction.

        // Initialize class members.
        // All via self-construction.
    }


    // ***********************************************************************
    // init
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    // For this overall class, we build out all the defaults for robot control
    @Override
    public void init() {
        List<VoltageSensor>  voltSensors;

        swerveDebug(500, "SwerveCore::init", "START");

        // Default format for our numbers
        swerveNumberFormat = new DecimalFormat("0.00");

        // init for autonomous variables
        crater = Boolean.TRUE;
        autoDelay = 0;

        // Record the starting time for this OpMode
        startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
        fullTime.reset();

        swerveDebug(500, "SwerveCore::init", "DATE done");

        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        /*
           * *********** NOTE *********
           * The names of these devices are loaded from the XML file on the phone.
           * We can use "adb push" to add files onto the phone, rather than typing them in.
           * The files on the phone live in /sdcard/FIRST/...
           *
           * For the Rev Control hub, link with 'adb connnect 192.168.43.1:5555'
         */

        // Motors for the wheels
        motorLeftFront = hardwareMap.dcMotor.get("LeftFrontM");
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        swerveDebugDevice(500, "Left Front Motor", motorLeftFront);
        motorRightFront = hardwareMap.dcMotor.get("RightFrontM");
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        swerveDebugDevice(500, "Right Front Motor", motorRightFront);
        motorLeftRear = hardwareMap.dcMotor.get("LeftRearM");
        motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);
        swerveDebugDevice(500, "Left Rear Motor", motorLeftRear);
        motorRightRear = hardwareMap.dcMotor.get("RightRearM");
        motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorRightRear.setDirection(DcMotor.Direction.REVERSE);
        swerveDebugDevice(500, "Right Rear Motor", motorRightRear);

        swerveDebug(500, "SwerveCore::init", "MOTORS connected");


        // Servos for the wheels
        servoLeftFront = hardwareMap.servo.get("LeftFrontS");
        swerveDebugDevice(500, "Left Front Servo", servoLeftFront);
        servoRightFront = hardwareMap.servo.get("RightFrontS");
        swerveDebugDevice(500, "Right Front Servo", servoRightFront);
        servoLeftRear = hardwareMap.servo.get("LeftRearS");
        swerveDebugDevice(500, "Left Rear Servo", servoLeftRear);
        servoRightRear = hardwareMap.servo.get("RightRearS");
        swerveDebugDevice(500, "Right Rear Servo", servoRightRear);
        // Older code planned on these being scaled to a limited range, which has changed (Jan 2020)
//        servoLeftFront.scaleRange(1.0/12,11.0/12);
//        servoRightFront.scaleRange(1.0/12,11.0/12);
//        servoLeftRear.scaleRange(1.0/12,11.0/12);
//        servoRightRear.scaleRange(1.0/12,11.0/12);

        //Game Mark Drop servo
        gameMarkDrop = hardwareMap.servo.get("gameMarkDrop");
        swerveDebugDevice(500, "Game Marker Drop", gameMarkDrop);

        // foundation grab servos
        foundationLeft = hardwareMap.servo.get("GrabLeft");
        swerveDebugDevice(500, "Left Foundation Servo", foundationLeft);
        foundationRight = hardwareMap.servo.get("GrabRight");
        swerveDebugDevice(500, "Right Foundation Servo", foundationRight);
        foundationRight.setDirection(Servo.Direction.REVERSE);


        swerveDebug(500, "SwerveCore::init", "SERVOS connected");


        // Battery power monitoring
        // We found a very useful link to the battery voltage here:
        //    https://www.reddit.com/r/FTC/comments/3odx26/is_it_possible_to_get_the_battery_voltage/
        //    Only now we use a list to skip the fixed name
        voltSensors = hardwareMap.getAll(VoltageSensor.class);
        if ( voltSensors.size() > 0 ) {
            batteryVoltSensor = voltSensors.get(0);
        }
        swerveDebugDevice(500,"Battery Voltage Sensor", batteryVoltSensor);

        // Rev has a built-in IMU for relative position information. The swerve drive uses the IMU.
        ourIMU = hardwareMap.get(BNO055IMU.class, "imu");

        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = imuCalibration; // from calibration sample
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // activate the parameters
        ourIMU.initialize(parameters);

        swerveDebug(500,"Inertial management unit", "connected");


        swerveDebug(500, "SwerveCore::init", "SENSORS connected");

        // Connect up the swerve drive
        swerveRightFront = new SwerveWheel(motorRightFront, servoRightFront);
        swerveLeftFront = new SwerveWheel(motorLeftFront, servoLeftFront);
        swerveRightRear = new SwerveWheel(motorRightRear, servoRightRear);
        swerveLeftRear = new SwerveWheel(motorLeftRear, servoLeftRear);
        ourSwerve = new SwerveDrive(swerveRightFront, swerveLeftFront, swerveLeftRear, swerveRightRear, 12, 12, ourIMU );
        swerveDebug(500, "SwerveCore::init", "swerve drive created");

        swerveDebug( 500, "SwerveCore::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    @Override
    public void start() {
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.

        swerveDebug(500, "SwerveCore::start", "START");

        super.start();

        // Now that the robot has really started, note this as the real start time
        resetStartTime();
        runTime.reset();

        swerveDebug(500, "SwerveCore::start", "DONE");
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // Performs any actions that are necessary while the OpMode is running.
    // The system calls this member repeatedly while the OpMode is running.
    @Override
    public void loop() {
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.

        // Core loop only needs to be logged for very high debug levels
        swerveDebug(5000, "LoopC", "SwerveCore::loop run");

        loopEndReporting();
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is disabled.
    // The system calls this member once when the OpMode is disabled.
    @Override
    public void stop() {
        // Nothing needs to be done for this OpMode.
        swerveDebug(500, "Stop", "SwerveCore::Stop run");

        super.stop();
    }


    // ***********************************************************************
    // loopEndReporting
    // ***********************************************************************
    // Perform any background updates.
    // Report on core robot status
    public void loopEndReporting() {
        // Loop reporting only needs to be logged for very high debug levels
        swerveDebug( 5000, "LoopC", "SwerveCore::loopEndReporting run");

        // Note run time
        swerveLog("1 Start", "Core started at " + startDate);
        swerveLog("2 Status", "running for " + runTime.toString());

        // Current controler values
        swerveLog("  CTL 1", controllerTelemetry(gamepad1));
        swerveLog("  CTL 2", controllerTelemetry(gamepad2));

        // Swerve status
        swerveLog( "X S1", ourSwerve.getGravXYZAccel());
        swerveLog( "X S2", ourSwerve.getMoveLog());
        swerveLog( "X S3", ourSwerve.getMoveAdjustLog());
        swerveLog( "X S4", ourSwerve.getAngleLog());
        swerveLog( "X S5", ourSwerve.getSpeedLog());
        swerveLog( "X S6", ourSwerve.getOrientLog());
        swerveLog( "X S7", ourSwerve.getAutoDriveLog());
    }


    // ***********************************************************************
    // controllerTelemetry
    // ***********************************************************************
    // String of all the current controller values
    public String controllerTelemetry(Gamepad myPad) {
        String leftTrigger;
        String leftBumper;
        String rightTrigger;
        String rightBumper;
        String leftRightPush;
        String abxy;
        String answer;

        if (myPad.left_trigger > minTriggerMove) {
            leftTrigger = " LT: " + myPad.left_trigger;
        } else {
            leftTrigger = "";
        }
        if (myPad.left_bumper) {
            leftBumper = " LB";
        } else {
            leftBumper = "";
        }
        if (myPad.right_trigger > minTriggerMove) {
            rightTrigger = " RT: " + myPad.right_trigger;
        } else {
            rightTrigger = "";
        }
        if (myPad.right_bumper) {
            rightBumper = " RB";
        } else {
            rightBumper = "";
        }
        if (myPad.left_stick_button) {
            leftRightPush = " LSP";
        } else {
            leftRightPush = "";
        }
        if (myPad.right_stick_button) {
            leftRightPush += " RSP";
        }
        if (myPad.a) {
            abxy = " A";
        } else {
            abxy = "";
        }
        if (myPad.b) {
            abxy += " B";
        }
        if (myPad.x) {
            abxy += " X";
        }
        if (myPad.y) {
            abxy += " Y";
        }

        answer = " LX: " + swerveNumberFormat.format(myPad.left_stick_x)
                + " LY: " + swerveNumberFormat.format(myPad.left_stick_y)
                + " RX: " + swerveNumberFormat.format(myPad.right_stick_x)
                + " RY: " + swerveNumberFormat.format(myPad.right_stick_y)
                + leftRightPush
                + leftTrigger + leftBumper
                + rightTrigger + rightBumper
                + abxy;

        return answer;
    }


    // ***********************************************************************
    // showAutonomousGoals
    // ***********************************************************************
    // Use telemetry to report on the autonomous goal settings
    public void showAutonomousGoals() {
        if (crater) {
            swerveLog("Z1", "Target is crater");
        } else {
            swerveLog("Z1", "Target is  depot ");
        }

        swerveLog("Z2", "Delay is " + autoDelay);
    }

    // ***********************************************************************
    // swerveSleep
    // ***********************************************************************
    // Go to sleep and wait, time is in milliseconds
    public void swerveSleep(long millis ) {
        double  startTime;
        double  now;
        long    delta;

        swerveDebug(500, "SwerveCore::swerveSleep", "START, requested time is " + millis + "ms");

        startTime = getRuntime();
        do {
            now = getRuntime();
            delta = (long)((now - startTime) * 1000);
        } while (delta < millis);

        swerveDebug(500, "SwerveCore::swerveSleep", "DONE, elapsed time is " + delta + "ms");
    }


    // ***********************************************************************
    // swerveDebugDevice
    // ***********************************************************************
    // Debugging messages for device map
    public void swerveDebugDevice(int myLevel, String myName, HardwareDevice myDevice) {
        String  message;

        message = myName + "== name: " + myDevice.getDeviceName()
                + ", connect: " + myDevice.getConnectionInfo()
                + ", version: " + myDevice.getVersion();

        // Show the debug as telemetry if set for that level of debug
        if ( debugLevel > myLevel ) {
            telemetry.addData("**DEBUG DEVICE**", message);
        }

        // Add debug data to the log...
        RobotLog.i("**DEBUG DEVICE** " + message);
    }


    // ***********************************************************************
    // swerveDebug
    // ***********************************************************************
    // Debugging messages
    public void swerveDebug(int myLevel, String myName, String myMessage ) {

        // Show the debug as telemetry if set for that level of debug
        if ( debugLevel > myLevel ) {
            telemetry.addData( "**DEBUG**: " + myName, myMessage );
        }


        // Add debug data to the log...
        // -- for very high levels of debug item, ONLY add if debugging at that level
        if ((1000 > myLevel) || (debugLevel > myLevel)) {
            RobotLog.i("**DEBUG** == " + myName + ": " + myMessage);
        }
    }


    // ***********************************************************************
    // swerveLog
    // ***********************************************************************
    // Log messages that are always shown
    public void swerveLog(String myName, String myMessage ) {

        // Show the message on the driver display
        telemetry.addData( myName, myMessage);

        // Add debug data to the log...w
        RobotLog.i( "LOG == " + myName + ": " + myMessage );
    }
}

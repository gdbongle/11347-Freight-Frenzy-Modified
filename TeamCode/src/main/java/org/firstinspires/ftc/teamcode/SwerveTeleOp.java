// ***********************************************************************
// SwerveTeleOp
// ***********************************************************************
// The tele-op mode for swerve robot operations

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// ***********************************************************************
// Definitions from Qualcomm code for OpMode recognition
// ***********************************************************************
@TeleOp(name="Swerve: 2-TeleOp 1.0", group="Swerve")
//@Disabled
public class SwerveTeleOp extends SwerveCore {
    // Note when we are approaching the end of the game
    Boolean inEndGame;

    enum autoScoring {
        DRIVE_FORWARD,
        EXTEND,
        INTAKE,
        TURN_LEFT,
        TURN_RIGHT,
        LANDER
    }
    private autoScoring curScoreState;

    // scale back speed for fine-tuned moves
    private double speedScale;

    // ***********************************************************************
    // SwerveTeleOp
    // ***********************************************************************
    // Constructs the class.
    // The system calls this member when the class is instantiated.
    public SwerveTeleOp() {

    }

    // ***********************************************************************
    // Init
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is enabled.
    // The system calls this member once when the OpMode is enabled.
    @Override
    public void init() {
        swerveDebug(500, "SwerveTeleOp::init", "START");

        // Run initialization of other parts of the class
        // Note that the class will connect to all of our motors and servos
        super.init();



        // We are just starting, so not in the end game yet...
        inEndGame = Boolean.FALSE;

        swerveDebug(500, "SwerveTeleOp::init", "DONE");
    }


    // ***********************************************************************
    // start
    // ***********************************************************************
    // Do first actions when the start command is given.
    // Called once when the OpMode is started.
    @Override
    public void start() {
        swerveDebug(500, "SwerveTeleOp::start", "START");

        // Call the super/base class start method.
        super.start();

        ourSwerve.curSwerveMode = SwerveDrive.swerveModes.SWERVE_DRIVER;

        swerveDebug(500, "SwerveTeleOp::start", "DONE");
    }


    // ***********************************************************************
    // loop
    // ***********************************************************************
    // State machine for autonomous robot control
    // Called continuously while OpMode is running
    @Override
    public void loop() {
//        double totalPower;
//        int endGameTime;

        swerveDebug(2000, "SwerveTeleOp::loop", "START");

        // set swerve drive orientation automation level based on driver request
        if (gamepad1.a) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_AUTO);
        }
        if (gamepad1.b) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVER);
        }
        if (gamepad1.x) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVE_ORIENT);
        }
        if (gamepad1.y) {
            ourSwerve.setSwerveMode(SwerveDrive.swerveModes.SWERVE_DRIVE_TURN);
        }

        // if either trigger, slow down
        if (( gamepad1.left_trigger > minTriggerMove ) ||
                ( gamepad1.right_trigger > minTriggerMove )) {
            speedScale = 0.35;
        } else {
            speedScale = 1.0;
        }

//        // if either bumper, grab foundation
//        if ( gamepad1.left_bumper || gamepad1.right_bumper ) {
//            foundationLeft.setPosition( 0.85 );
//            foundationRight.setPosition( 0.85 );
//        } else {
//            foundationLeft.setPosition( 0.5 );
//            foundationRight.setPosition( 0.5 );
//        }

        // Move the robot, flipping y since the joysticks are upside down
        ourSwerve.driveRobot(( gamepad1.left_stick_x * speedScale ),
                -( gamepad1.left_stick_y * speedScale ),
                ( gamepad1.right_stick_x * speedScale ),
                -( gamepad1.right_stick_y * speedScale ));

        // Any loop background updates happen now....
        loopEndReporting();

        swerveDebug(500, "SwerveTeleOp::loop", "DONE");
    }


    // ***********************************************************************
    // stop
    // ***********************************************************************
    // Performs any actions that are necessary when the OpMode is disabled.
    // The system calls this member once when the OpMode is disabled.
    @Override
    public void stop() {
        swerveDebug(500, "SwerveTeleOp::stop", "START");

        // Call the super/base class stop method
        super.stop();

        swerveDebug(500, "SwerveTeleOp::stop", "DONE");
    }
}
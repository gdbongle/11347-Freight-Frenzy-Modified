// ***********************************************************************
// SwerveWheel
// ***********************************************************************
// Controls for one of four wheels on a swerve drive robot.
// Each wheel is a single unit that has both a motor (speed) and a servo (position).
//
// Speed is 0 to 1 (never negative). Motor configuration needs to have positive as moving forward.
//
// Servo postion is -1 to 1. Servo configuration needs to be set so that the motion range is
// from -180 to 180, with 0 as straight.
//
// Because we are building a servo-based robot, we chose to implement the code below using a
// range of -90 to 90. We flip the sign of the speed when we adapt from values outside this
// target range.
//
// *** DERIVED FROM ***
// GREAT data on swerve drive design found here: https://www.chiefdelphi.com/media/papers/2426
// Posted by Ether starting in 2011. The simple calculations were clear. The spreadsheet model
// was an amazing help for visualizing what we were building before driving it.
//
// We also found some useful code ideas posted on GitHub by FTC team 1251 (references to
// team1251.org not working as of April, 2018)
// The code was downloaded from https://github.com/bob80333/swerve-drive
// That code was created by Eric Engelhart on 3/20/2017

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// ***********************************************************************
// SwerveWheel
// ***********************************************************************
// Class definitions
public class SwerveWheel {
    DcMotor motor;
    private Servo servo;
    private double oldPosition;
    // for the 270 degree servo range, or half range of 135, 75% of 180 degrees
    // we use slightly less since the servos are not perfect
    //
    // since the servo can go more than 180 degrees, there are variable areas
    // between ~45 and ~135 degrees on each side which can be used to drive
    // the robot at the intended angle (just flip the wheel direction). We use
    // that to allow the robot to change from ~-135 through 0 to ~135 without
    // the need to 'flip' the servo around. Once we flip the wheel direction,
    // the same works for ~-45  through +/-180 to ~45. This minimizes the
    // frequency of flipping, which can mis-position the robot in
    // autonomous - especially since the flip by default happens right at the
    // +-90 angle for going exactly sideways.
    //
    // NOTE: true 270 would be 75, but 70 tests correctly on Rev servos
    private double SERVO_LIMIT = 0.70;  // not 75, based on Rev tests
    private double SERVO_MAX = 0.70;
    private double SERVO_MIN = -0.70;
    private double SERVO_MAX_FLIP = -0.30;
    private double SERVO_MIN_FLIP = 0.30;

    // ***********************************************************************
    // SwerveWheel - create a new swerve wheel unit
    // ***********************************************************************
    // Creates a new instance of a wheel, using the motor and servo provided.
    public SwerveWheel( DcMotor useMotor, Servo useServo ){
        this.motor  = useMotor;
        this.servo = useServo;
        oldPosition = 0.0;

        // stop any movement
        updateWheel( 0, 0);
    }

    // ***********************************************************************
    // updateWheel - apply motor power and turn the servo
    //
    // The servo can move 270 degrees
    // For up to +/- 45 degrees, the speed has to be forward
    // For below -135 or above 135, the speed has to be backward
    // In between, we may flip the speed to minimize the servo change
    // ***********************************************************************

    public void updateWheel(double newSpeed, double newPosition){
        // be sure we have a valid new position
        if(( newPosition < -1.0 ) || ( newPosition > 1.0 )) {
            newPosition = 0;
        }
        // be sure we have a valid and useful speed
        if ( newSpeed < 0 ) {
            newSpeed = 0;
        }

        // based on new position desired and old servo target, set new target
        // if below servo min, flip speed and angle
        if ( newPosition < SERVO_MIN ) {
            newSpeed = -newSpeed;
            newPosition = newPosition + 1.0;
        }
        // if above servo max, flip speed and angle
        if ( newPosition > SERVO_MAX ) {
            newSpeed = -newSpeed;
            newPosition = newPosition - 1.0;
        }

        // now check both sideways options to see if a sign flip minimizes servo rotation
        if (( newPosition >= SERVO_MIN ) && ( newPosition < ( SERVO_MAX_FLIP ))) {
            if (( oldPosition - newPosition ) > ( newPosition + 1.0 - oldPosition )) {
                newSpeed = -newSpeed;
                newPosition = newPosition + 1.0;
            }
        } else if (( newPosition <= SERVO_MAX ) && ( newPosition > ( SERVO_MIN_FLIP ))) {
            if (( oldPosition - newPosition ) < ( newPosition - 1.0 - oldPosition )) {
                newSpeed = -newSpeed;
                newPosition = newPosition - 1.0;
            }
        }

        // set the motor power
        motor.setPower( newSpeed );
        // testing servo controls only....
//        motor.setPower( 0 );

        // if the wheel is moving, use the servo to set the position
        if (( newSpeed != 0 )) {
            // servo position is based on 0 to 1, not -1 to 1
            servo.setPosition(( newPosition / ( SERVO_LIMIT * 2 )) + 0.5 );
            // save the old position
            oldPosition = newPosition;
        }
    }
}

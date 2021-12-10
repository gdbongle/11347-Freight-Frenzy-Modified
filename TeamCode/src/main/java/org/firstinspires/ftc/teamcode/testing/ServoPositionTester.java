package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo and Motor Tester")
public class ServoPositionTester extends LinearOpMode {


    Servo leftFront;
    Servo leftBack;
    Servo rightFront;
    Servo rightBack;

    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightRear;
    DcMotor motorLeftRear;

    boolean testingServos = true;
    boolean wasPressed = false;

    private void setPositions(double position){
        leftFront.setPosition(position);
        leftBack.setPosition(position);
        rightFront.setPosition(position);
        rightBack.setPosition(position);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(Servo.class, "LeftFrontS");
        leftBack = hardwareMap.get(Servo.class, "LeftRearS");
        rightFront = hardwareMap.get(Servo.class, "RightFrontS");
        rightBack = hardwareMap.get(Servo.class, "RightRearS");


        motorLeftFront = hardwareMap.dcMotor.get("LeftFrontM");
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRightFront = hardwareMap.dcMotor.get("RightFrontM");
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLeftRear = hardwareMap.dcMotor.get("LeftRearM");
        motorLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRightRear = hardwareMap.dcMotor.get("RightRearM");
        motorRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()){

            if(gamepad1.a && !wasPressed){
                testingServos = !testingServos;
            }
            wasPressed = gamepad1.a;


            if(testingServos){
                telemetry.addData("Mode", "Servos");
                telemetry.addLine("DPAD LEFT for left position");
                telemetry.addLine("DPAD UP for middle position");
                telemetry.addLine("DPAD RIGHT for right position");
                if (gamepad1.dpad_up) {
                    setPositions(0.5);
                } else if (gamepad1.dpad_left) {
                    setPositions(0);
                } else if (gamepad1.dpad_right) {
                    setPositions(1);
                }
            } else {
                telemetry.addData("Mode", "Motors");
                telemetry.addLine("Left bumper for left front");
                telemetry.addLine("Left trigger for left back");
                telemetry.addLine("Right bumper for right front");
                telemetry.addLine("Right trigger for right back");
                if(gamepad1.left_bumper) motorLeftFront.setPower(0.2);
                else motorLeftFront.setPower(0);
                if(gamepad1.left_trigger > 0.2) motorLeftRear.setPower(0.2);
                else motorLeftRear.setPower(0);
                if(gamepad1.right_bumper) motorRightFront.setPower(0.2);
                else motorRightFront.setPower(0);
                if(gamepad1.right_trigger > 0.2) motorRightRear.setPower(0.2);
                else motorRightRear.setPower(0);
            }

            telemetry.update();
        }

    }
}

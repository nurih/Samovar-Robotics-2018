package org.firstinspires.ftc.teamcode;

import android.text.method.MovementMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous(name = "autonamouse v1")
public class autonamous extends LinearOpMode {

    private static final double DRIVE_MOTOR_POWER = 1;
    private  static final double ARM_MOTOR_POWER = 1;

    double WANTED_MOVEMENT_L = 3;

    int wantedMovement = 1120;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;
    @Override
    public void runOpMode() throws InterruptedException {
        initializeArmMotor(ARM_MOTOR_POWER);

        initializeLeftMotor(DRIVE_MOTOR_POWER, WANTED_MOVEMENT_L);

        initializeRightMotor(DRIVE_MOTOR_POWER);

        waitforstart();
        //pull arm motor back
        int currentPositionA = arm1Motor.getCurrentPosition();
        int wantedMovementA = 1200;

        telemetry.addLine("Started");
        telemetry.update();

        int wantedMovement = 500;
        autanamous_drive(wantedMovement);
        wantedMovement = 50;
        int currentPositionL = leftMotor.getCurrentPosition();
        //determine next position
        int determinedPositionL = wantedMovement + currentPositionL;
        //move
        leftMotor.setPower(1);
        leftMotor.setTargetPosition(determinedPositionL);
        while (leftMotor.isBusy()) {
            telemetry.addData("PositionL", leftMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotor.setPower(0);
        //go forward from here
        wantedMovement = 500;
        autanamous_drive(wantedMovement);



    }




    private void waitforstart() {
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
    }


    private void left_telemetry(int currentPositionL, int determinedPositionL) {
        telemetry.addData("Current L is", currentPositionL);
        telemetry.addData("Desired L is", determinedPositionL);
        telemetry.update();
    }
    private void right_telemetry(int currentPositionR, int determinedPositionR) {
        telemetry.addData("Current R is", currentPositionR);
        telemetry.addData("Desired R is", determinedPositionR);
        telemetry.update();
    }

    private void initializeArmMotor(double ARM_MOTOR_POWER) {
        arm1Motor = hardwareMap.dcMotor.get("arm1motor");
        arm1Motor.setDirection(DcMotor.Direction.FORWARD);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setPower(0.1);
    }
    private void initializeRightMotor(double driveMotorPower) {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(driveMotorPower);
        int currentPositionL = leftMotor.getCurrentPosition();
        telemetry.addLine("right motor ready");
    }

    private void initializeLeftMotor(double driveMotorPower, double WANTED_MOVEMENT_L) {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(driveMotorPower);
        telemetry.addLine("left motor ready");
    }

    private void autanamous_drive(int wantedMovement) {
        //get current position
        int currentPositionL = leftMotor.getCurrentPosition();
        int currentPositionR = rightMotor.getCurrentPosition();
        //determine next position


        int determinedPositionL = wantedMovement + currentPositionL;
        int determinedPositionR = wantedMovement + currentPositionR;

        left_telemetry(currentPositionL, determinedPositionL);

        right_telemetry(currentPositionR, determinedPositionR);

        //Move motor
        leftMotor.setTargetPosition(determinedPositionL);
        rightMotor.setTargetPosition(determinedPositionR);
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
            telemetry.addData("PositionL", leftMotor.getCurrentPosition());
            telemetry.addData("PositionR", rightMotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(1000);
        telemetry.addData("PositionL", leftMotor.getCurrentPosition());
        telemetry.addData("PositionR", rightMotor.getCurrentPosition());
        telemetry.addLine("Done");
        telemetry.update();
    }
}

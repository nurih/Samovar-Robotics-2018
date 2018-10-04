package org.firstinspires.ftc.teamcode;

import android.text.method.MovementMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Autonomous(name = "autonamouse v1")
public class autonamous extends LinearOpMode {

    private static final double DRIVE_MOTOR_POWER = 1;
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {


        initializeLeftMotor(DRIVE_MOTOR_POWER);

        initializeRightMotor(DRIVE_MOTOR_POWER);

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        //get current position
        int currentPositionL = leftMotor.getCurrentPosition();
        int currentPositionR = rightMotor.getCurrentPosition();
        //determine next position
        int wantedMovement = 1140;

        int determinedPositionL = wantedMovement + currentPositionL;
        int determinedPositionR = wantedMovement + currentPositionR;

        left_telemetry(currentPositionL, determinedPositionL);

        telemetry.addData("Current R is", currentPositionR);
        telemetry.addData("Desired R is", determinedPositionR);
        telemetry.update();

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

    private void left_telemetry(int currentPositionL, int determinedPositionL) {
        telemetry.addData("Current L is", currentPositionL);
        telemetry.addData("Desired L is", determinedPositionL);
        telemetry.update();
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

    private void initializeLeftMotor(double driveMotorPower) {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(driveMotorPower);
        telemetry.addLine("left motor ready");
    }
}

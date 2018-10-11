package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "autonamouse v1")
public class Autonomous1OpMode extends LinearOpMode {


    //inches
    private static final double DRIVE_MOTOR_POWER = 1;
    private static final double ARM_MOTOR_POWER = 1;
    private static final int TURN_ANGLE_90_DEGREES = 90;
    private static final double DRIVE_TO_WALL = 48;

    double WANTED_MOVEMENT_L = 3;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        waitforstart();
        initializeHardware();

        telemetry.addLine("Started");
        telemetry.update();

        pullArmMotorBack();

        goStraight(DRIVE_TO_WALL);
        waitForDriveDone();

        turn(TurnDirection.RIGHT, TURN_ANGLE_90_DEGREES);

        waitForDriveDone();


    }

    private void waitForDriveDone() {
        while (leftMotor.isBusy() || rightMotor.isBusy()) {
            // spin wait.
        }

    }

    private void turn(TurnDirection turnDirection, int turnAngleDegrees) {
        DcMotor motor;

        motor = (turnDirection == TurnDirection.RIGHT) ? leftMotor : rightMotor;

        int currentPositionLeft = motor.getCurrentPosition();

        motor.setTargetPosition(currentPositionLeft + degreesToTicks(turnAngleDegrees));
    }

    private void initializeHardware() {
        arm1Motor = initializeArmMotor(ARM_MOTOR_POWER);
        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, "leftMotor", DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, "rightMotor", DcMotor.Direction.FORWARD);
    }

    private void pullArmMotorBack() {
        // TODO: move arm motor
    }


    private void waitforstart() {
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
    }


    private DcMotor initializeArmMotor(double power) {
        DcMotor arm1Motor = hardwareMap.dcMotor.get("arm1motor");
        arm1Motor.setDirection(DcMotor.Direction.FORWARD);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setPower(power);
        return arm1Motor;
    }


    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(driveMotorPower);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }

    private static int inchToTicks(double inches) {
        return (int) (inches * (1120 / 46));

    }

    private static int degreesToTicks(double degrees) {
        double circumference = 2 * Math.PI * 13.5;
        double arclength = (circumference/360)*degrees;
        return inchToTicks(arclength);
    }

    private void goStraight(double distanceInches) {

        int distanceTicks = inchToTicks(distanceInches);

        int currentPositionLeft = leftMotor.getCurrentPosition();
        int currentPositionRight = rightMotor.getCurrentPosition();
        telemetry.addData("PositionL", currentPositionLeft);
        telemetry.addData("PositionR", currentPositionRight);


        leftMotor.setTargetPosition(distanceTicks + currentPositionLeft);
        rightMotor.setTargetPosition(distanceTicks + currentPositionRight);

        telemetry.addData("PositionL", leftMotor.getCurrentPosition());
        telemetry.addData("PositionR", rightMotor.getCurrentPosition());

        telemetry.update();
    }
}

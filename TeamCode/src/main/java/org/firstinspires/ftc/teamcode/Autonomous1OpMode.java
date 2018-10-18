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
    private static final int TURN_ANGLE_45_DEGREES = 45;
    private static final double DRIVE_TO_WALL = 48;
    private static final double DRIVE_FROM_LATCH = 3;

    double WANTED_MOVEMENT_L = 3;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;
    public static final int DETACH_FROM_LANDER_TICKS = 5652;

    @Override
    public void runOpMode() throws InterruptedException {
        waitforstart();
        initializeHardware();

        telemetry.addLine("Started");
        telemetry.update();

        lowerFromLatch();
        sleep(1000);

        goStraight(DRIVE_FROM_LATCH);
        sleep(1000);

        turn(TurnDirection.RIGHT,TURN_ANGLE_45_DEGREES);

        sleep(1000);

        goStraight(DRIVE_TO_WALL);

        waitForDriveDone();
        sleep(1000);

        turn(TurnDirection.RIGHT, -TURN_ANGLE_90_DEGREES);

        waitForDriveDone();
        sleep(1000);

        goStraight(DRIVE_TO_WALL);

        waitForDriveDone();
        sleep(1000);

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

    private void lowerFromLatch() {
        int currentPosition = arm1Motor.getCurrentPosition();

        int determinedPosition = currentPosition + DETACH_FROM_LANDER_TICKS;
        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();
        //Move motor
        arm1Motor.setTargetPosition(determinedPosition);
        while (arm1Motor.isBusy() && isStarted()) {
            telemetry.addData("Position", arm1Motor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Position", arm1Motor.getCurrentPosition());
        telemetry.addLine("Done");
        telemetry.update();
    }


    private void waitforstart() {
        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();
    }


    private DcMotor initializeArmMotor(double power) {
        DcMotor arm1Motor = hardwareMap.dcMotor.get(RobotPart.ARM_1_MOTOR);
        arm1Motor.setDirection(DcMotor.Direction.REVERSE);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
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

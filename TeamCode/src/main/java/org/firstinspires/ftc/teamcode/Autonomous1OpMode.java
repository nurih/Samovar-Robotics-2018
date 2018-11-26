package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonomous V1")
@Disabled
public class Autonomous1OpMode extends Teapot {

    //inches
    private static final double DRIVE_MOTOR_POWER = 1;
    private static final double ARM_MOTOR_POWER = 1;
    private static final int TURN_ANGLE_90_DEGREES = 90;
    private static final int TURN_ANGLE_45_DEGREES = 45;
    private static final double DRIVE_TO_WALL = 48;
    private static final double DRIVE_FROM_LATCH = 5;
    public static final int DETACH_FROM_LANDER_TICKS = 8700;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor arm1Motor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        lowerFromLatch();
        sleep(1000);

        goStraight(DRIVE_FROM_LATCH);
        sleep(1000);

        turn(TurnDirection.RIGHT, TURN_ANGLE_45_DEGREES);
        sleep(1000);

        goStraight(DRIVE_TO_WALL);
        sleep(1000);

        turn(TurnDirection.RIGHT, -TURN_ANGLE_90_DEGREES);
        sleep(1000);

        goStraight(DRIVE_TO_WALL);
        sleep(1000);

    }


    private void turn(TurnDirection turnDirection, int turnAngleDegrees) {
        DcMotor motor;

        motor = (turnDirection == TurnDirection.RIGHT) ? leftMotor : rightMotor;

        int currentPositionLeft = motor.getCurrentPosition();

        motor.setTargetPosition(currentPositionLeft + Teapot.degreesToTicks(turnAngleDegrees));

        waitForDriveDone(leftMotor, rightMotor);
    }

    private void initializeHardware() {
        arm1Motor = initializeArmMotor(ARM_MOTOR_POWER);
        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
        telemetry.addLine("Initialized Hardware");
        telemetry.update();
    }

    private void lowerFromLatch() {
        int currentPosition = arm1Motor.getCurrentPosition();

        int determinedPosition = currentPosition + DETACH_FROM_LANDER_TICKS;
        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();
        //Move motor
        arm1Motor.setTargetPosition(determinedPosition);
        while (arm1Motor.isBusy() && !isStopRequested()) {
            telemetry.addData("Position", arm1Motor.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Position", arm1Motor.getCurrentPosition());
        telemetry.addLine("Done");
        telemetry.update();
    }


    private DcMotor initializeArmMotor(double power) {
        DcMotor arm1Motor = hardwareMap.dcMotor.get(RobotPart.ARM_1_MOTOR);
        arm1Motor.setPower(0);
        arm1Motor.setDirection(DcMotor.Direction.REVERSE);
        arm1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1Motor.setTargetPosition(arm1Motor.getCurrentPosition());
        arm1Motor.setPower(power);
        telemetry.addLine("Arm initialized");
        return arm1Motor;
    }


    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(driveMotorPower);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }


    private void goStraight(double distanceInches) {
        int distanceTicks = Teapot.inchToTicks(distanceInches);

        leftMotor.setTargetPosition(distanceTicks + leftMotor.getCurrentPosition());
        rightMotor.setTargetPosition(distanceTicks + rightMotor.getCurrentPosition());

        waitForDriveDone(leftMotor, rightMotor);
    }

}

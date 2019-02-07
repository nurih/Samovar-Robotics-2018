package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ImuDrive2 {
    public static final double DEGREES_TOLERANCE = 5;
    public static final double DRIVE_MOTOR_POWER = 0.5;
    public static final double TURN_MOTOR_POWER = 0.3;
    private final HardwareMap hardwareMap;

    final Telemetry telemetry;
    final LinearOpMode opMode;

    DcMotor leftMotor;
    DcMotor rightMotor;

    IMU imu;

    public ImuDrive2(LinearOpMode parentOpMode) {

        this.hardwareMap = parentOpMode.hardwareMap;
        this.telemetry = parentOpMode.telemetry;
        this.opMode = parentOpMode;

        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
        imu = new IMU(hardwareMap, telemetry);
    }

    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);

        telemetry.addLine(motorName + " motor ready");
        return motor;
    }

    public double turn(int targetAngleUniversal) {
        double measuredAngle = 0;

        while (this.opMode.opModeIsActive() && !angleIsReached(targetAngleUniversal)) {
            measuredAngle = getDegrees();

            if (measuredAngle < targetAngleUniversal) {
                rightMotor.setPower(TURN_MOTOR_POWER);
                leftMotor.setPower(-TURN_MOTOR_POWER);
            } else if (measuredAngle > targetAngleUniversal) {
                rightMotor.setPower(-TURN_MOTOR_POWER);
                leftMotor.setPower(TURN_MOTOR_POWER);
            } else {
                rightMotor.setPower(0);
                leftMotor.setPower(0);
            }
        }
        return measuredAngle;
    }

    public double getDegrees() {
        return imu.getTurningDegrees();
    }

    public void straight(double distanceInches) {

        int distanceTicks = Teapot.inchToTicks(distanceInches);

        int currentPositionLeft = leftMotor.getCurrentPosition();
        int currentPositionRight = rightMotor.getCurrentPosition();

        int targetTicksLeft = currentPositionLeft + distanceTicks;
        int targetTicksRight = currentPositionRight + distanceTicks;

        // Math.signum is just the sign: negative 1 or 1 or zero
        double directionalPower = DRIVE_MOTOR_POWER * Math.signum(distanceTicks);
        showStatus(targetTicksLeft, targetTicksRight, directionalPower);
        telemetry.update();
        do {
            rightMotor.setPower(directionalPower);
            leftMotor.setPower(directionalPower);
            showStatus(targetTicksLeft, targetTicksRight, directionalPower);

            telemetry.update();
        }
        while (opMode.opModeIsActive()
                && !targetIsReached(leftMotor, targetTicksLeft)
                && !targetIsReached(rightMotor, targetTicksRight));

        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }

    private void showStatus(int targetTicksLeft, int targetTicksRight, double directionalPower) {
        telemetry.addData("Directional Power", directionalPower);
        telemetry.addData("Left is", leftMotor.getCurrentPosition());
        telemetry.addData("=> ", targetTicksLeft);
        telemetry.addData("Right is", rightMotor.getCurrentPosition());
        telemetry.addData("=> ", targetTicksRight);
    }

    public boolean targetIsReached(DcMotor motor, int targetTicks) {
        int ticksFromTarget = Math.abs(Math.abs(motor.getCurrentPosition()) - Math.abs(targetTicks));
        telemetry.addData("Target Ticks Distance", ticksFromTarget);
        return ticksFromTarget < 12;
    }

    private boolean angleIsReached(double targetAngle) {
        double measuredAngle = getDegrees();
        return angleIsReached(targetAngle, measuredAngle);
    }

    private boolean angleIsReached(double targetAngle, double measuredAngle) {
        return Math.abs(Math.abs(targetAngle) - Math.abs(measuredAngle)) <= DEGREES_TOLERANCE;
    }
}

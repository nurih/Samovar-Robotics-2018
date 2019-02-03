package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ImuDrive {
    public static final double DEGREES_TOLERANCE = 5;
    public static final double DRIVE_MOTOR_POWER = 1;
    public static final int TICKS_DELTA = 10;
    private final HardwareMap hardwareMap;

    final Telemetry telemetry;
    final LinearOpMode opMode;

    DcMotor leftMotor;
    DcMotor rightMotor;

    IMU imu;

    public ImuDrive(LinearOpMode parentOpMode) {

        this.hardwareMap = parentOpMode.hardwareMap;
        this.telemetry = parentOpMode.telemetry;
        this.opMode = parentOpMode;

        leftMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.LEFT_MOTOR, DcMotor.Direction.REVERSE);
        rightMotor = initializeDriveMotor(DRIVE_MOTOR_POWER, RobotPart.RIGHT_MOTOR, DcMotor.Direction.FORWARD);
        imu = new IMU(hardwareMap, telemetry);
    }

    public void waitForMotorsToReach() {
        while (
                leftMotor.isBusy() && rightMotor.isBusy()) {
            // spinning, do nothing, stay in loop

        }
        return;
//        while (
//                (!
//
//                        (leftMotor.getCurrentPosition()>=leftMotor.getTargetPosition()-10)
//                        && (leftMotor.getCurrentPosition()<= leftMotor.getTargetPosition()+10)
//                        &&(leftMotor.getCurrentPosition()>=leftMotor.getTargetPosition()-10)
//                        && (rightMotor.getCurrentPosition()<= rightMotor.getTargetPosition()+10)
//                        &&(rightMotor.getCurrentPosition()>= rightMotor.getTargetPosition()-10)))
//        {
//            //WE WAITN
//        }
    }

    private DcMotor initializeDriveMotor(double driveMotorPower, String motorName, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        // prevent any rogue movement from previous initializations while configuring.
        motor.setPower(0);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(motor.getCurrentPosition());

        motor.setPower(driveMotorPower);
        telemetry.addLine(motorName + " motor ready");
        return motor;
    }

    public double turn(int targetAngleUniversal) {
        double measuredAngle = 0;

        while (this.opMode.opModeIsActive() && !angleIsReached(targetAngleUniversal)) {
            measuredAngle = getDegrees();

            if (measuredAngle < targetAngleUniversal - DEGREES_TOLERANCE) {
                rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + TICKS_DELTA);
                leftMotor.setTargetPosition(leftMotor.getCurrentPosition() - TICKS_DELTA);
            } else if (measuredAngle > targetAngleUniversal + DEGREES_TOLERANCE) {
                rightMotor.setTargetPosition(rightMotor.getCurrentPosition() - TICKS_DELTA);
                leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + TICKS_DELTA);
            } else {
                // do nothing. When drive-to-position, it should stay in place
            }
        }
        return measuredAngle;
    }

    public double getDegrees() {
        return imu.getDegrees();
    }

    public void straight(double distanceInches) {

        int distanceTicks = Teapot.inchToTicks(distanceInches);

        int currentPositionLeft = leftMotor.getCurrentPosition();
        int currentPositionRight = rightMotor.getCurrentPosition();

        leftMotor.setTargetPosition(distanceTicks + currentPositionLeft);
        rightMotor.setTargetPosition(distanceTicks + currentPositionRight);

    }

    public boolean angleIsReached(double targetAngle) {
        double measuredAngle = getDegrees();
        return angleIsReached(targetAngle, measuredAngle);
    }

    public boolean angleIsReached(double targetAngle, double measuredAngle) {
        // within 1 degree
        return Math.abs(targetAngle - measuredAngle) <= DEGREES_TOLERANCE;
    }
}

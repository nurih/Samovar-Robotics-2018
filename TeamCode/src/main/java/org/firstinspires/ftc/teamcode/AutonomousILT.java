package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Autonomous2OpMode.DETACH_FROM_LANDER_TICKS;

public abstract class AutonomousILT extends Teapot {
    static final double AWAY_FROM_HOOK = 3;

    DcMotor arm1Motor = null;
    ImuDrive imuDrive;
    SamoVision samoVision;

    protected void initializeAndWaitForStart() {
        arm1Motor = initializeArmMotor(1);
        imuDrive = new ImuDrive(this);

        samoVision = new SamoVision(hardwareMap);
        Say("Hardware initialized");
        waitForStart();
    }

    protected void turnTowardsGoldMineral() {
        //loop where we turn, and check if we are seeing gold thingy\
        double startingAngle = imuDrive.getDegrees();
        Say("Starting angle", startingAngle);
        for (int attempt = 1; attempt <= 6; attempt++) {
            if (samoVision.trySeeGoldThing()) {
                Say("It's right there!!! attempt", attempt);
                break;
            } else {
                int targetAngle = (int) startingAngle + 12 * attempt;
                Say("Not seeing it. Turning to", targetAngle);
                imuDrive.turn(targetAngle);
                pause();
            }
        }
    }

    protected void knockOffGoldThingy() {
        int attempts = 0;
        while (!this.isStopRequested()) {
            // vision - try to see object.
            if (samoVision.trySeeGoldThing()) {

                double angle = samoVision.getEstimatedAngle();
                if (Math.abs(angle) > 2) {
                    imuDrive.turn((int) (imuDrive.getDegrees() + angle));
                }

                imuDrive.straight(-4);
                imuDrive.waitUntilPositionReached();
                telemetry.addLine("Inched forward");
                attempts++;
            } else {
                if (attempts > 8) {
                    break;
                }
            }
            telemetry.update();
        }
    }

    protected void pause() {
        sleep(500);
    }

    protected void pullArmBack() {
        int currentPosition = arm1Motor.getCurrentPosition();
        int determinedPosition = currentPosition - (DETACH_FROM_LANDER_TICKS);

        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();
        //Move motor
        arm1Motor.setTargetPosition(determinedPosition);

    }

    protected void lowerFromLatch() {
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

    }

    protected void driveToCrater() {
        // capture current floor angles to calibrate if we are tilted based on
        // this field floor
        imuDrive.imu.calibrateTilt();

        for (int attempt = 0; attempt < 6; attempt++) {
            imuDrive.straight(-4);
            imuDrive.waitUntilPositionReached();
            if (imuDrive.imu.isTilted()) {
                break;
            }
            Say("Going to crater ", attempt);
        }

    }

    protected DcMotor initializeArmMotor(double power) {
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

}



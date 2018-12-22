package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Autonomous2OpMode.DETACH_FROM_LANDER_TICKS;

@Autonomous(name = "Autonomous Meet3 V1")
public class AutonomousMeet3v1 extends Teapot {

    DcMotor arm1Motor = null;
    ImuDrive imuDrive;
    SamoVision samoVision;

    @Override
    public void runOpMode() throws InterruptedException {
        arm1Motor = initializeArmMotor(1);
        imuDrive = new ImuDrive(this);

        samoVision = new SamoVision(hardwareMap);
        telemetry.addLine("Hardware initialized");
        telemetry.update();
        waitForStart();

        lowerFromLatch();

        pause();
        imuDrive.straight(-6);

        pause();
        double currentAngle = imuDrive.turn(-45);
        telemetry.addData("Current angle is ", currentAngle);
        telemetry.update();
        pause();
        knockOffGoldThingy();

        telemetry.addLine("Done");
        telemetry.update();
    }

    private void knockOffGoldThingy() {
        while (!this.isStopRequested()) {
            // vision - try to see object.
            if (samoVision.trySeeGoldThing()) {
                double angle = samoVision.getEstimatedAngle();
                if (Math.abs(angle) > 2) {
                    imuDrive.turn((int) (imuDrive.getDegrees() + angle));
                }

                imuDrive.straight(-10);
                telemetry.addLine("Inched forward");
            } else {
                telemetry.addLine("Not seeing it...");
            }
            telemetry.update();
        }


    }

    private void pause() {
        sleep(1000);
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

}



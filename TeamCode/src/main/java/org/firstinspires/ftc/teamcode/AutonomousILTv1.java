package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Autonomous2OpMode.DETACH_FROM_LANDER_TICKS;

@Autonomous(name = "Autonomous ILT V1")
public class AutonomousILTv1 extends Teapot {

    DcMotor arm1Motor = null;
    ImuDrive imuDrive;
    SamoVision samoVision;

    @Override
    public void runOpMode() throws InterruptedException {
        arm1Motor = initializeArmMotor(1);
        imuDrive = new ImuDrive(this);

        samoVision = new SamoVision(hardwareMap);
        Say("Hardware initialized");
        waitForStart();

        // off the wall
        lowerFromLatch();

        double AWAY_FROM_HOOK = 3;
        // away from hook to detach
        imuDrive.straight(AWAY_FROM_HOOK);
        pause();

        // retract latch arm
        pullArmBack();
        pause();

        // back forward so not bumping against lander leg
        imuDrive.straight(-AWAY_FROM_HOOK);
        pause();

        // facing out of lander
        imuDrive.turn(60);
        Say("Turned out to field");
        pause();

        // into field to get closer to minerals
        imuDrive.straight(-6);
        Say("Drove away from lander");
        pause();

        // try out luck to get it the first time
        samoVision.trySeeGoldThing();

        //loop where we turn, and check if we are seeing gold thingy\
        double startingAngle = imuDrive.getDegrees();
        Say("Starting angle", startingAngle);
        for (int attempt = 1; attempt <= 5; attempt++) {
            if (samoVision.trySeeGoldThing()) {
                Say("It's right there!!! attempt");
                break;
            } else {
                int targetAngle = (int) startingAngle + 12 * attempt;
                Say("Not seeing it. Turning to", targetAngle);
                imuDrive.turn(targetAngle);
                pause();
            }
        }

        Say("Here we go?");
        knockOffGoldThingy();

        imuDrive.straight(6);
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
        sleep(500);
    }

    private void pullArmBack() {
        int currentPosition = arm1Motor.getCurrentPosition();
        int determinedPosition = currentPosition - (DETACH_FROM_LANDER_TICKS);

        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();
        //Move motor
        arm1Motor.setTargetPosition(determinedPosition);

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



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
        telemetry.addLine("Hardware initialized");
        telemetry.update();
        waitForStart();

        lowerFromLatch();
        pause();
        imuDrive.straight(2 );

        pause();
        backinit();
        //base turn
        // 45
        //loop where we turn, and check if we are seeing gold thingy\
        imuDrive.turn(60);
        pause();
        for(int x = 60; x <= 121; x+=20 ){
            telemetry.addLine("The x =");
            telemetry.addData("X:",x);
            if(!samoVision.trySeeGoldThingyForTest()){
                telemetry.addLine("not seeing it");
                imuDrive.straight(-2);
                imuDrive.turn(x);
                pause();
            }else{
                telemetry.addLine("seeing it");

                break;
            }
            telemetry.addLine("here we go?");

        }
        telemetry.update();
        pause();
        knockOffGoldThingy();
        pause();
        imuDrive.straight(5);
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

    private void backinit(){
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



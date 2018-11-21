package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "armV1")
public class armV1 extends LinearOpMode {
    DcMotor arm1motor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeArmMotor();

        //arm1motor.setPower(0.5);

        telemetry.addLine("arm Motor ready");

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        telemetry.addLine("Started");
        telemetry.update();

        //get current position
        int currentPosition = arm1motor.getCurrentPosition();

        //determine next position
        int wantedMovement = 1140;

        int determinedPosition = wantedMovement + currentPosition;

        telemetry.addData("Current is", currentPosition);
        telemetry.addData("Desired is", determinedPosition);
        telemetry.update();

        //Move motor
        arm1motor.setTargetPosition(determinedPosition);
        while (arm1motor.isBusy()) {
            telemetry.addData("Position", arm1motor.getCurrentPosition());
            telemetry.update();
        }
        arm1motor.setPower(0);
        sleep(1000);
        telemetry.addData("Position", arm1motor.getCurrentPosition());
        telemetry.addLine("Done");
        telemetry.update();

    }

    private void initializeArmMotor() {
        arm1motor = hardwareMap.dcMotor.get("arm1motor");
        arm1motor.setPower(0);
        arm1motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm1motor.setDirection(DcMotor.Direction.FORWARD);
        // pull up
        arm1motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1motor.setPower(0.1);
    }
}

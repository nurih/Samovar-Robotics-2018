package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CollectorExtenderServo {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final CRServo servo;



    public CollectorExtenderServo(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.servo = hardwareMap.crservo.get(RobotPart.COLLECTOR_EXTENDER_SERVO);
        this.servo.setDirection( DcMotorSimple.Direction.FORWARD);
        stop();
    }

    public void rotate(double power){
            servo.setPower( power);
    }


    public void stop(){
        this.servo.setPower(0);
    }
}

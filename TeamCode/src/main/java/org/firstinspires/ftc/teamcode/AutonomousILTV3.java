package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ILT Off Wall Only")
public class AutonomousILTV3 extends AutonomousILT{

    @Override
    public void runOpMode() throws InterruptedException {
        initializeAndWaitForStart();

        // off the wall
        lowerFromLatch();

        // away from hook to detach
        imuDrive.straight(AWAY_FROM_HOOK);
        pause();

        // retract latch arm
        pullArmBack();


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

    }

}

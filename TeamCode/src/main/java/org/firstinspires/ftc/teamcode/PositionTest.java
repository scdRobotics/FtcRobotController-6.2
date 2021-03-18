package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="PositionTest", group="linearOpMode")
public class PositionTest extends AutonomousPrime2020 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();

        //telemetry.addData("Hello!", " World!");
        //pause(10);
        /*leftEncoder(90, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(180, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(270, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(360, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);*/

        VennisFunctEnhanced(200, 0.25, 0.5);

        //wobbleGrabDown(1);
        //pause(100);

        /*latch.setPosition(0);
        pause(2);
        latch.setPosition(0.1);
        pause(2);
        latch.setPosition(0.2);
        pause(2);
        latch.setPosition(0.3);
        pause(2);
        latch.setPosition(0.4);
        pause(2);
        latch.setPosition(0.5);
        pause(2);
        latch.setPosition(0.6);
        pause(2);
        latch.setPosition(0.7);
        pause(2);
        latch.setPosition(0.8);
        pause(2);
        latch.setPosition(0.9);
        pause(2);
        latch.setPosition(1);
        pause(2);*/

    }
}
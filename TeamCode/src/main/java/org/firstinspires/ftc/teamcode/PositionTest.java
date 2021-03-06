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

        double test = 5;

        //strafeRightEncoder(10, 1);

        //strafeRightDistCheck(0.25);
        //telemetry.addData("test",test);
        //pause(2);
        //strafeLeftEncoder(5, 0.5);
        while(test==5){
            updateDist();
            //strafeLeftEncoder(5, 0.5);
        }


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
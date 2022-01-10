package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="EnocderTestthing")

public class EncoderTestthing extends LinearOpMode {

    minibot robot = new minibot();


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.encoderForwardDrive(0.2,12,10,this);
        //robot.encoderSideDrive(0.2,12,10,this);

    }

}

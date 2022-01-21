package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="DuckParkBlue")

public class DuckParkBlue extends LinearOpMode {

    minibot robot = new minibot();


    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, this);

        waitForStart();

        robot.encoderSideDrive(-0.3,-3,7,this);
        sleep(500);
        robot.encoderForwardDrive(0.3,4,4,this);
        sleep(250);
        robot.spin.setPower(0.575);
        sleep(3000);
        robot.spin.setPower(0);
        robot.encoderForwardDrive(-0.3,-5,7,this);
        sleep(250);
        robot.encoderSideDrive(-0.3,-24,15,this);
        sleep(250);
        robot.encoderForwardDrive(0.3,20,15,this);

    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



    @Autonomous(name="ParkL")

    public class ParkL extends LinearOpMode {

        minibot robot = new minibot();


        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap, this);
            robot.claw.setPosition(0.8);


            waitForStart();

            robot.motorFront.setPower(0.35);
            robot.motorBack.setPower(-0.35);
            sleep(2000);
            robot.motorBack.setPower(0);
            robot.motorFront.setPower(0);
            robot.encoderForwardDrive(0.3,18,4,this);



        }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



    @Autonomous(name="Park")

    public class Park extends LinearOpMode {

        minibot robot = new minibot();


        public void runOpMode() throws InterruptedException {
            robot.init(hardwareMap, this);

            waitForStart();

            robot.motorFront.setPower(0.35);
            robot.motorBack.setPower(-0.35);
            sleep(3000);
            robot.motorBack.setPower(0);
            robot.motorFront.setPower(0);


        }

}

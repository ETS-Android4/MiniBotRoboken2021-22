package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="RedDuckSIDE")

public class RedDuckSIDE extends LinearOpMode {


        minibot robot = new minibot();


        public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap, this);
            robot.claw.setPosition(0.8);
            waitForStart();

            robot.arm.setPower(-0.3);
            sleep(2000);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25, 21, 5, this);
            sleep(1000);

            if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {

            robot.encoderSideDrive(0.25, 16, 5, this);

            robot.arm.setPower(0.2);
            sleep(600);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25,2,2,this);

            robot.claw.setPosition(0);

            robot.arm.setPower(-0.4);
            sleep(500);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25, -24, 5, this);
            robot.encoderSideDrive(0.25, 60, 7, this);
            }
            else {
                robot.encoderSideDrive(0.25,-9,5,this);
                sleep(1000);
                if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {

                    robot.encoderSideDrive(0.25, 27, 5, this);

                    robot.encoderForwardDrive(0.25,-4.2,3,this);

                    robot.arm.setPower(0.2);
                    sleep(800);
                    robot.arm.setPower(-0.1);
                    sleep(300);
                    robot.encoderForwardDrive(0.25,2.4,2,this);
                    robot.claw.setPosition(0);
                    sleep(300);

                    robot.encoderForwardDrive(0.25, -24, 5, this);
                    sleep(300);
                    robot.arm.setPower(-0.4);
                    sleep(500);
                    robot.arm.setPower(0);
                    sleep(300);
                    robot.encoderSideDrive(0.25, 60, 7, this);
                    }
                else {
                    robot.encoderSideDrive(0.25, 25, 5, this);

                    robot.encoderForwardDrive(0.25,-6.7,3,this);

                    robot.arm.setPower(0.2);
                    sleep(950);
                    robot.arm.setPower(-0.1);
                    sleep(300);
                    robot.encoderForwardDrive(0.25,5.5,2,this);
                    robot.claw.setPosition(0);
                    sleep(300);

                    robot.encoderForwardDrive(0.25, -24, 5, this);
                    sleep(300);
                    robot.arm.setPower(-0.4);
                    sleep(500);
                    robot.arm.setPower(0);
                    sleep(300);
                    robot.encoderSideDrive(0.25, 60, 7, this);
                }

            }
    }


}

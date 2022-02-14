package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="BlueDuckSIDE")

public class BlueDuckSIDE extends LinearOpMode {


        minibot robot = new minibot();


        public void runOpMode() throws InterruptedException {

            robot.init(hardwareMap, this);
            robot.claw.setPosition(0.8);
            waitForStart();

            robot.arm.setPower(-0.3);
            sleep(1800);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25, 21, 5, this);

            if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {

            robot.encoderSideDrive(0.25, 18, 5, this);

            robot.arm.setPower(0.2);
            sleep(600);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25,2,2,this);

            robot.claw.setPosition(0);

            robot.arm.setPower(-0.4);
            sleep(500);
            robot.arm.setPower(0);

            robot.encoderForwardDrive(0.25, -21, 5, this);
            robot.encoderSideDrive(0.25, -60, 7, this);
            }
            else {
                robot.encoderSideDrive(0.25,-9,5,this);
                if(robot.distanceSensor.getDistance(DistanceUnit.CM) < 10) {

                    robot.encoderSideDrive(0.25, 27, 5, this);

                    robot.encoderForwardDrive(0.25,-4.2,3,this);

                    robot.arm.setPower(0.2);
                    sleep(900);
                    robot.arm.setPower(-0.1);
                    sleep(300);
                    robot.encoderForwardDrive(0.25,1.6,2,this);
                    robot.claw.setPosition(0);
                    sleep(300);

                    robot.encoderForwardDrive(0.25, -21, 6, this);
                    sleep(300);
                    robot.arm.setPower(-0.4);
                    sleep(500);
                    robot.arm.setPower(0);
                    sleep(300);
                    robot.encoderSideDrive(0.25, -60, 7, this);
                    }
                else {
                    robot.encoderSideDrive(0.25, 27, 5, this);

                    robot.encoderForwardDrive(0.25,-5.5,3,this);

                    robot.arm.setPower(0.2);
                    sleep(1025);
                    robot.arm.setPower(-0.1);
                    sleep(300);
                    robot.encoderForwardDrive(0.25,4.5,2,this);
                    robot.claw.setPosition(0);
                    sleep(300);

                    robot.encoderForwardDrive(0.25, -21, 5, this);
                    sleep(300);
                    robot.arm.setPower(-0.4);
                    sleep(500);
                    robot.arm.setPower(0);
                    sleep(300);
                    robot.encoderSideDrive(0.25, -60, 7, this);
                }

            }
    }


}

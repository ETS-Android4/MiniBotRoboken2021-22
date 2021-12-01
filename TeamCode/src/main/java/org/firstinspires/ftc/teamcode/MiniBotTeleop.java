package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
@TeleOp(name="MiniTeleop")

public class MiniBotTeleop extends LinearOpMode{


    private DcMotor motorFront;
    private DcMotor motorBack;
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private double speedControl = 0.5;
    @Override


    public void runOpMode()
    {

       motorFront = hardwareMap.dcMotor.get("motorFront");
       motorBack = hardwareMap.dcMotor.get("motorBack");
       motorLeft = hardwareMap.dcMotor.get("motorLeft");
       motorRight = hardwareMap.dcMotor.get("motorRight");



        waitForStart();

        while (opModeIsActive()){

            if(gamepad1.right_stick_y!=0||gamepad1.right_stick_x!=0) {
                motorLeft.setPower(gamepad1.right_stick_y*speedControl);
                motorRight.setPower(-gamepad1.right_stick_y*speedControl);
                motorFront.setPower(-gamepad1.right_stick_x*speedControl);
                motorBack.setPower(gamepad1.right_stick_x*speedControl);
            }
           else if(gamepad1.left_stick_x!=0||gamepad1.left_stick_y!=0){
                motorLeft.setPower(-gamepad1.left_stick_x*speedControl);
                motorRight.setPower(-gamepad1.left_stick_x*speedControl);
                motorFront.setPower(-gamepad1.left_stick_x*speedControl);
                motorBack.setPower(-gamepad1.left_stick_x*speedControl);
            }
           else{

               motorLeft.setPower(0);
               motorRight.setPower(0);
               motorFront.setPower(0);
               motorBack.setPower(0);
            }


           if (gamepad1.dpad_up) speedControl=1.0;
           if (gamepad1.dpad_left) speedControl=0.5;
           if (gamepad1.dpad_right) speedControl=0.5;
           if (gamepad1.dpad_down) speedControl=0.25;






    }}}
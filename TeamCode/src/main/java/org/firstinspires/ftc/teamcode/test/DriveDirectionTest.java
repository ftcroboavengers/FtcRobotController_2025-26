package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;


    @TeleOp(name="Drive Direction Test")
    public class DriveDirectionTest extends LinearOpMode {
        @Override
        public void runOpMode() {
            DcMotor fl = hardwareMap.dcMotor.get("front_left_drive");
            DcMotor fr = hardwareMap.dcMotor.get("front_right_drive");
            DcMotor bl = hardwareMap.dcMotor.get("back_left_drive");
            DcMotor br = hardwareMap.dcMotor.get("back_right_drive");

            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.FORWARD);

            waitForStart();
            while (opModeIsActive()) {
                double y = -gamepad1.left_stick_y; // forward
                double x = gamepad1.left_stick_x; // strafe right
                double r = gamepad1.right_stick_x; // rotate CW

                double flp = y + x + r;
                double frp = y - x - r;
                double blp = y - x + r;
                double brp = y + x - r;

                fl.setPower(flp);
                fr.setPower(frp);
                bl.setPower(blp);
                br.setPower(brp);
            }
        }
    }



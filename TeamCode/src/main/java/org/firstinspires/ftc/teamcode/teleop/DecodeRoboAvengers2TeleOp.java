package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Decode RoboAvengers (PIDF)", group = "RoboAvengers")
public class DecodeRoboAvengers2TeleOp extends LinearOpMode {

    // ----------------- Drive / Pinpoint -----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private boolean fieldCentric = false;
    private double headingOffsetRad = 0.0;

    // ----------------- Intake / Launch -----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private boolean rightBumperPressed = false;
    private boolean rightLauncherOn = false;
    private boolean leftBumperPressed = false;
    private boolean leftLauncherOn = false;

    // ----------------- Launcher Targets -----------------
    private static final double LAUNCH_CLOSE_TARGET = 800;
    private static final double LAUNCH_FAR_TARGET = 800;
    private double launcherTarget = LAUNCH_CLOSE_TARGET;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Map drive motors ----
        leftFront = firstMotor("front_left_drive", "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack = firstMotor("back_left_drive",  "backLeftMotor");
        rightBack = firstMotor("back_right_drive", "backRightMotor");

        intake = getMotor("intake");
        leftLauncher = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        if (leftLauncher != null) {
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // PIDF tuning
            // If shots oscillate or overshoot, lower the P or D values.
            // If shots underpower, raise the F (feedforward) slightly — e.g. from 12.0 → 13.0.
            leftLauncher.setVelocityPIDFCoefficients(30.0, 0.0, 10.0, 12.0);
        }

        if (rightLauncher != null) {
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // PIDF tuning
            rightLauncher.setVelocityPIDFCoefficients(30.0, 0.0, 10.0, 12.0);
        }

        // ---- Intake directions ----
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Motor directions ----
        if (leftFront != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack  != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront!= null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // ---- Pinpoint setup ----
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();
            zeroHeading();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — field-centric disabled.");
            fieldCentric = false;
        }

        telemetry.addLine("RoboAvengers TeleOp READY. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --------------- DRIVE ---------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.x) fieldCentric = !fieldCentric;
            if (gamepad1.y) zeroHeading();

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);
            mecanumDrive(y, x, rx, driveScale);

            // --------------- INTAKE ---------------
            if (intake != null) {
                double in1 = gamepad1.right_trigger;
                double out1 = gamepad1.left_trigger;
                double in2 = gamepad2.right_trigger;
                double out2 = gamepad2.left_trigger;

                // Combine input from both gamepads
                // whichever is pressed more
                double in = Math.max(in1, in2);
                double out = Math.max(out1, out2);

                double p = 0;
                double scale = 1;
                 if (in > 0.01 || out > 0.01) {
                    p = (in - out) * scale;
                }

                intake.setPower(p);
            }

            // --------------- LAUNCHERS ---------------
            if (leftLauncher != null && rightLauncher != null) {
                // Distance presets
                if (gamepad2.a) launcherTarget = LAUNCH_CLOSE_TARGET;
                if (gamepad2.b) launcherTarget = LAUNCH_FAR_TARGET;

                // --- Right bumper toggle ---
                if (gamepad2.right_bumper && !rightBumperPressed) {
                    rightLauncherOn = !rightLauncherOn;
                    if (rightLauncherOn) {
                        startRightLauncher();
                    } else {
                        stopRightLauncher();
                    }
                }
                rightBumperPressed = gamepad2.right_bumper;

                // --- Left bumper toggle ---
                if (gamepad2.left_bumper && !leftBumperPressed) {
                    leftLauncherOn = !leftLauncherOn;
                    if (leftLauncherOn) {
                        startLeftLauncher();
                    } else {
                        stopLeftLauncher();
                    }
                }
                leftBumperPressed = gamepad2.left_bumper;

                if (gamepad2.x) {
                    stopLaunchers();
                    rightLauncherOn = false;
                    leftLauncherOn = false;
                }
            }

            // --------------- TELEMETRY ---------------
            if (pinpoint != null) {
                pinpoint.update();
                double yaw = getYawRad();
                telemetry.addData("Heading(deg)", Math.toDegrees(yaw));
                telemetry.addData("FieldCentric", fieldCentric);
                telemetry.addData("X(mm)", pinpoint.getPosX(DistanceUnit.MM));
                telemetry.addData("Y(mm)", pinpoint.getPosY(DistanceUnit.MM));
            }
            telemetry.addData("Launcher Target", launcherTarget);
            telemetry.addData("Left Vel", leftLauncher.getVelocity());
            telemetry.addData("Right Vel", rightLauncher.getVelocity());
            telemetry.addData("Target Vel", launcherTarget);
            telemetry.update();
        }

        stopLaunchers();
        setPower(0, 0, 0, 0);
    }

    // ----------------- Helpers -----------------
    private void mecanumDrive(double y, double x, double rx, double scale) {
        if (leftFront == null) return;
        double rotX = x;
        double rotY = y;
        if (fieldCentric && pinpoint != null) {
            double yaw = getYawRad();
            double cosA = Math.cos(-yaw);
            double sinA = Math.sin(-yaw);
            rotX = x * cosA - y * sinA;
            rotY = x * sinA + y * cosA;
        }

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double fl = (rotY + rotX + rx) / denom * scale;
        double bl = (rotY - rotX + rx) / denom * scale;
        double fr = (rotY - rotX - rx) / denom * scale;
        double br = (rotY + rotX - rx) / denom * scale;
        setPower(fl, fr, bl, br);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack != null) leftBack.setPower(bl);
        if (rightBack != null) rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) if (m != null) m.setZeroPowerBehavior(BRAKE);
    }

    private void zeroHeading() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            headingOffsetRad = 0.0;
        }
    }

    private double getYawRad() {
        if (pinpoint == null) return 0.0;
        return pinpoint.getHeading(AngleUnit.RADIANS) - headingOffsetRad;
    }

    private void startRightLauncher() {
        double adjustedVelocity = getVoltageCompensatedVelocity(launcherTarget);
        rightLauncher.setVelocity(adjustedVelocity);
    }

    private void startLeftLauncher() {
        double adjustedVelocity = getVoltageCompensatedVelocity(launcherTarget);
        leftLauncher.setVelocity(adjustedVelocity);
    }

    private void stopLaunchers() {
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    private void stopLeftLauncher() {
        leftLauncher.setPower(0);
    }

    private void stopRightLauncher() {
        rightLauncher.setPower(0);
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0; // fully charged battery
        double currentVoltage = 12.0;
        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception e) {
            telemetry.addLine("Voltage sensor not found — using 12V default");
        }
        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    // -------- Safe hardware getters --------
    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); } catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); } catch (Exception e) { return null; }
    }
}

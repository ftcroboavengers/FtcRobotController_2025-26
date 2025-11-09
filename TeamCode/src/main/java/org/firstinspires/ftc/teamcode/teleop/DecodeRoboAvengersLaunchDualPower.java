package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Decode RoboAvengers (ALT)", group = "RoboAvengers")
public class DecodeRoboAvengersLaunchDualPower extends LinearOpMode {

    // ---------------- Drive / Pinpoint ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private boolean fieldCentric = true;
    private double headingOffsetRad = 0.0;

    // ---------------- Intake / Launch ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- Launcher Power Settings ----------------
    private static double LEFT_LAUNCH_POWER  = 0.7;
    private static double RIGHT_LAUNCH_POWER = 0.7;

    // ---------------- Timings ----------------
    private static final double SPINUP_TIME_SEC   = 2.0;
    private static final double FEED_TIME_SEC     = 1.0;
    private static final double REVERSE_TIME_SEC  = 1.0;
    private static final double STOP_DELAY_SEC    = 0.25;
    private static final double FEED_POWER        = 1.0;

    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState  = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    private ElapsedTime leftTimer  = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Map hardware ----
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // ---- Drive setup ----
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // ---- Intake ----
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Launcher setup ----
        if (leftLauncher != null) {
            leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightLauncher != null) {
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ---- Pinpoint ----
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

        telemetry.addLine("TeleOp READY — Dual Power Launchers");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --------------- DRIVE ---------------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.x) fieldCentric = !fieldCentric;
            if (gamepad1.y) zeroHeading();

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);
            mecanumDrive(y, x, rx, driveScale);

            // --------------- INTAKE ---------------
            if (intake != null && leftState == LaunchState.IDLE && rightState == LaunchState.IDLE) {
                double in1 = gamepad1.right_trigger;
                double out1 = gamepad1.left_trigger;
                double in2 = gamepad2.right_trigger;
                double out2 = gamepad2.left_trigger;
                double in = Math.max(in1, in2);
                double out = Math.max(out1, out2);
                double p = 0;
                if (out > 0.01) p = -out;
                else if (in > 0.01) p = in;
                intake.setPower(p);
            }

            // --------------- CLEAR / STOP ---------------
            if (gamepad2.y) clearAll();

            // --------------- SHOOT + UNJAM ---------------
            boolean unjamPressed = gamepad2.x;
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // --------------- UNJAM MANUAL ---------------
            if (gamepad2.x
                    && leftState == DecodeRoboAvengersLaunchDualPower.LaunchState.IDLE
                    && rightState == DecodeRoboAvengersLaunchDualPower.LaunchState.IDLE) {
                startReverseUnjam();
            }

            // --------------- Telemetry ---------------
            if (pinpoint != null) {
                pinpoint.update();
                telemetry.addData("Heading(deg)", Math.toDegrees(getYawRad()));
                telemetry.addData("FieldCentric", fieldCentric);
            }
            telemetry.addData("Left State", leftState);
            telemetry.addData("Right State", rightState);
            telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0);
            telemetry.update();
        }

        stopLaunchers();
        setPower(0, 0, 0, 0);
    }

    // ---------------- Helpers ----------------
    private void clearAll() {
        stopLaunchers();
        if (intake != null) intake.setPower(0);
        leftTimer.reset();
        rightTimer.reset();
        leftState = rightState = LaunchState.IDLE;
        telemetry.addLine("CLEAR ALL triggered — reset complete.");
        telemetry.update();
    }

    private void mecanumDrive(double y, double x, double rx, double scale) {
        if (leftFront == null) return;
        double rotX = x, rotY = y;
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
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors)
            if (m != null) m.setZeroPowerBehavior(BRAKE);
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

    // ---------------- Launcher Control ----------------
    private void startLaunchers() {
        if (leftLauncher != null)  leftLauncher.setPower(LEFT_LAUNCH_POWER);
        if (rightLauncher != null) rightLauncher.setPower(RIGHT_LAUNCH_POWER);
    }

    private void stopLaunchers() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private void startReverseUnjam() {
        if (intake != null) intake.setPower(0);
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        leftState = rightState = LaunchState.REVERSE;
        leftTimer.reset();
        rightTimer.reset();
    }

    // ---------------- State Machines ----------------
    private void launchLeft(boolean shootRequested, boolean unjamRequested) {
        if (leftLauncher == null || intake == null) return;
        switch (leftState) {
            case IDLE:
                if (shootRequested) {
                    leftTimer.reset();
                    startLaunchers();
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (leftTimer.seconds() > SPINUP_TIME_SEC) {
                    intake.setPower(FEED_POWER);
                    leftTimer.reset();
                    leftState = LaunchState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                if (unjamRequested) {
                    intake.setPower(0);
                    stopLaunchers();
                    startReverseUnjam();
                    break;
                }
                if (leftTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    leftTimer.reset();
                    leftState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (unjamRequested) { startReverseUnjam(); break; }
                if (leftTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunchers();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (leftTimer.seconds() > REVERSE_TIME_SEC) {
                    stopLaunchers();
                    leftState = rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    private void launchRight(boolean shootRequested, boolean unjamRequested) {
        if (rightLauncher == null || intake == null) return;
        switch (rightState) {
            case IDLE:
                if (shootRequested) {
                    rightTimer.reset();
                    startLaunchers();
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (rightTimer.seconds() > SPINUP_TIME_SEC) {
                    intake.setPower(FEED_POWER);
                    rightTimer.reset();
                    rightState = LaunchState.LAUNCHING;
                }
                break;

            case LAUNCHING:
                if (unjamRequested) { startReverseUnjam(); break; }
                if (rightTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    rightTimer.reset();
                    rightState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (unjamRequested) { startReverseUnjam(); break; }
                if (rightTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunchers();
                    rightState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (rightTimer.seconds() > REVERSE_TIME_SEC) {
                    stopLaunchers();
                    leftState = rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ---------------- Safe Getters ----------------
    private DcMotor firstMotor(String primary, String alt) {
        DcMotor m = getMotor(primary);
        if (m == null) m = getMotor(alt);
        return m;
    }
    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }
    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}

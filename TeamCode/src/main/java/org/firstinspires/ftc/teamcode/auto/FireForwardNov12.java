package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Forward + Fire", group = "RoboAvengers")
public class FireForwardNov12 extends LinearOpMode {

    // ---------------- Drive and Launch Motors ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;
    private GoBildaPinpointDriver pinpoint;

    // ---------------- Motion Tuning ----------------
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double MAX_TURN_POWER  = 0.4;
    private static final double kDrive = 0.025;
    private static final double kTurn  = 0.012;
    private static final double TURN_MIN_POWER = 0.20;

    // ---------------- Launcher Power Settings ----------------
    private static double LEFT_LAUNCH_POWER  = 0.75;
    private static double RIGHT_LAUNCH_POWER = 0.75;

    // ---------------- State Machine ----------------
    private enum AutoState { FIRE, FORWARD, DONE }
    private AutoState state = AutoState.FIRE;
    private AutoState lastState = null;

    private int stableCount = 0;
    private boolean xForwardIsPositive = true;
    private double lastX = 0;
    private long wrongWayCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Map Hardware ---
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");
        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // --- Drive Directions ---
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // --- Launcher Setup ---
        if (leftLauncher != null) {
            leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (rightLauncher != null) {
            rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // --- Pinpoint Setup ---
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();

            telemetry.addLine("Waiting for Pinpoint to settle...");
            for (int i = 0; i < 30; i++) {
                pinpoint.update();
                telemetry.addData("X", pinpoint.getPosX(DistanceUnit.INCH));
                telemetry.update();
                sleep(30);
            }
            pinpoint.resetPosAndIMU();
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — continuing without odometry.");
            telemetry.update();
            sleep(1000);
            pinpoint = null;
        }

        telemetry.addLine("READY: Forward + Fire (No Twitch)");
        telemetry.addData("Left Launch Power", LEFT_LAUNCH_POWER);
        telemetry.addData("Right Launch Power", RIGHT_LAUNCH_POWER);
        telemetry.update();

        waitForStart();

        // --- Axis Sanity Check (moved here, after Start, no Init twitch) ---
        if (pinpoint != null) {
            pinpoint.update();
            double x0 = pinpoint.getPosX(DistanceUnit.INCH);

            // ⚠️ Use small forward test motion AFTER start
            setPower(0.2, 0.2, 0.2, 0.2);
            sleep(300);
            stopDrive();

            pinpoint.update();
            double dx = pinpoint.getPosX(DistanceUnit.INCH) - x0;
            xForwardIsPositive = (dx >= 0);
            pinpoint.resetPosAndIMU();

            telemetry.addData("Axis check", "forwardIsPositiveX=%s (dx=%.2f)", xForwardIsPositive, dx);
            telemetry.update();
        }

        // --- Main Loop ---
        while (opModeIsActive() && state != AutoState.DONE) {
            if (pinpoint != null) pinpoint.update();

            if (state != lastState) {
                resetRuntime();
                stableCount = 0;
                lastState = state;
            }

            switch (state) {
                case FIRE:
                    telemetry.addLine("State: FIRE (6 s total)");
                    startLaunchers();
                    sleep(3000);
                    if (intake != null) intake.setPower(0.8);
                    sleep(3000);
                    if (intake != null) intake.setPower(0);
                    stopLaunchers();
                    stopDrive();
                    state = AutoState.FORWARD;
                    break;

                case FORWARD:
                    telemetry.addLine("State: FORWARD 30 in");
                    // 🔁 Positive means forward — make sure sign matches your setup
                    if (moveToX(mapForwardInchesToPinpointX(30.0)) || getRuntime() > 4.0) {
                        stopDrive();
                        sleep(500);
                        state = AutoState.DONE;
                    }
                    break;

                default:
                    state = AutoState.DONE;
                    break;
            }

            if (pinpoint != null) {
                telemetry.addData("X (in)", "%.1f", pinpoint.getPosX(DistanceUnit.INCH));
                telemetry.addData("Y (in)", "%.1f", pinpoint.getPosY(DistanceUnit.INCH));
                telemetry.addData("Heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
            }
            telemetry.addData("State", state);
            telemetry.addData("StableCount", stableCount);
            telemetry.update();

            idle();
        }

        stopLaunchers();
        stopDrive();
        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // ---------------- Motion Helpers ----------------
    private double mapForwardInchesToPinpointX(double forwardInches) {
        // Ensures positive means forward
        return xForwardIsPositive ? forwardInches : -forwardInches;
    }

    private boolean moveToX(double targetXInches) {
        if (pinpoint == null) return true;
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double error = targetXInches - currentX;
        double power = kDrive * error;
        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));
        setPower(power, power, power, power);

        double dx = currentX - lastX;
        if (Math.abs(power) > 0.05 && Math.abs(dx) > 0.01) {
            boolean goingOpposite = Math.signum(power) != Math.signum(dx);
            wrongWayCount = goingOpposite ? wrongWayCount + 1 : 0;
            if (wrongWayCount > 15) {
                stopDrive();
                wrongWayCount = 0;
                return true;
            }
        } else {
            wrongWayCount = 0;
        }
        lastX = currentX;

        return Math.abs(error) < 2.0;
    }

    private double normalize180(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private boolean turnToHeading(double targetDeg) {
        double current = normalize180(pinpoint.getHeading(AngleUnit.DEGREES));
        double error = normalize180(targetDeg - current);

        double power = kTurn * error;
        if (Math.abs(power) < TURN_MIN_POWER && Math.abs(error) > 2)
            power = Math.copySign(TURN_MIN_POWER, power);
        power = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, power));

        setPower(-power, power, -power, power);

        telemetry.addData("Turn Target", targetDeg);
        telemetry.addData("Current", current);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);

        boolean onTarget = Math.abs(error) < 5.0;
        if (onTarget) stableCount++;
        else stableCount = 0;
        return stableCount > 10;
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

    // ---------------- Drive Utilities ----------------
    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void stopDrive() { setPower(0, 0, 0, 0); }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors)
            if (m != null) m.setZeroPowerBehavior(BRAKE);
    }

    // ---------------- Safe Hardware Getters ----------------
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

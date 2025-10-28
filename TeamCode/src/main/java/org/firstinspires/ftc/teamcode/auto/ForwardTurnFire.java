package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * State machine auto:
 *  - FORWARD: drive forward ~48 in (encoders)
 *  - TURN: rotate in place 180 degrees (encoders)
 *  - FIRE: spin launchers + intake for 5 sec
 *  - STOP: shutdown
 */
@Autonomous(name = "Forward 48 → Turn 180 → Fire (State Machine)", group = "RoboAvengers")
public class ForwardTurnFire extends LinearOpMode {

    // Drive
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // Mechanisms
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // --- Motion constants (tune these to your robot) ---
    private static final double TICKS_PER_REV = 537.6;     // goBILDA 312rpm (change if using 435/600)
    private static final double WHEEL_DIAM_IN = 3.78;      // measure your wheel OD
    private static final double GEAR_RATIO    = 1.0;       // external reduction to wheels
    private static final double TICKS_PER_IN  = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAM_IN);

    // Track width = distance between left and right wheel contact patches (inches)
    // Measure hub-to-hub across the robot and tune for perfect turning.
    private static final double TRACK_WIDTH_IN = 14.5;

    private static final double MOVE_POWER = 0.5;
    private static final double TURN_POWER = 0.5;

    // Launcher velocity (from your TeleOp)
    private static final double LAUNCH_TARGET = 1200;

    // State machine
    private enum AutoState { FORWARD, TURN, FIRE, STOP, DONE }
    private AutoState state = AutoState.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware map (supports your two naming schemes) ---
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");

        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Directions to match your TeleOp
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        telemetry.addLine("READY: Forward 48 → Turn 180 → Fire 5s");
        telemetry.addData("Ticks/In", TICKS_PER_IN);
        telemetry.addData("Track Width (in)", TRACK_WIDTH_IN);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && state != AutoState.DONE) {
            switch (state) {

                case FORWARD:
                    telemetry.addLine("State: FORWARD 48 in");
                    telemetry.update();
                    driveForwardInches(48.0, MOVE_POWER, 6.0);
                    state = AutoState.TURN;
                    break;

                case TURN:
                    telemetry.addLine("State: TURN 180°");
                    telemetry.update();
                    // Positive degrees = clockwise (turn right). You can flip sign if you want CCW.
                    turnDegreesCW(180.0, TURN_POWER, 5.0);
                    state = AutoState.FIRE;
                    break;

                case FIRE:
                    telemetry.addLine("State: FIRE (5s)");
                    telemetry.update();
                    startLaunchers();
                    sleep(300); // brief spin-up (optional)
                    if (intake != null) intake.setPower(1.0);
                    sleep(5000);
                    if (intake != null) intake.setPower(0.0);
                    stopLaunchers();
                    state = AutoState.STOP;
                    break;

                case STOP:
                    telemetry.addLine("State: STOP");
                    telemetry.update();
                    setPower(0, 0, 0, 0);
                    state = AutoState.DONE;
                    break;

                default:
                    state = AutoState.DONE;
                    break;
            }

            idle();
        }

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // ----------------- Movement helpers -----------------

    private void driveForwardInches(double inches, double power, double timeoutSec) {
        if (!driveAvailable()) {
            // Fallback timed move
            setPower(power, power, power, power);
            sleep(1500);
            setPower(0, 0, 0, 0);
            return;
        }

        resetAllEncoders();

        int ticks = (int) Math.round(inches * TICKS_PER_IN);
        // With your directions, positive target should drive the robot forward.
        leftFront.setTargetPosition(ticks);
        rightFront.setTargetPosition(ticks);
        leftBack.setTargetPosition(ticks);
        rightBack.setTargetPosition(ticks);

        runToPositionAll(Math.abs(power));

        double start = getRuntime();
        while (opModeIsActive() && anyBusy() && (getRuntime() - start) < timeoutSec) {
            telemetry.addData("Forward", "%d / %d", leftFront.getCurrentPosition(), ticks);
            telemetry.update();
            idle();
        }

        stopAndRunUsingEncoder();
    }

    /**
     * Turn in place clockwise by 'degrees' using encoders.
     * Wheel travel per side = PI * TRACK_WIDTH_IN * (degrees / 360).
     * Left goes forward, Right goes backward for CW.
     */
    private void turnDegreesCW(double degrees, double power, double timeoutSec) {
        if (!driveAvailable()) {
            // Fallback timed spin (power left forward, right backward)
            setPower(power, -power, power, -power);
            sleep(1200); // rough guess for 180° — tune or replace with IMU later
            setPower(0, 0, 0, 0);
            return;
        }

        resetAllEncoders();

        double inchesPerSide = Math.PI * TRACK_WIDTH_IN * (degrees / 360.0);
        int ticks = (int) Math.round(inchesPerSide * TICKS_PER_IN);

        // For CW: left side +ticks (forward), right side -ticks (backward)
        leftFront.setTargetPosition(+ticks);
        leftBack.setTargetPosition(+ticks);
        rightFront.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        runToPositionAll(Math.abs(power));

        double start = getRuntime();
        while (opModeIsActive() && anyBusy() && (getRuntime() - start) < timeoutSec) {
            telemetry.addData("Turn CW", "LF:%d RF:%d LB:%d RB:%d",
                    leftFront.getCurrentPosition(), rightFront.getCurrentPosition(),
                    leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }

        stopAndRunUsingEncoder();
    }

    // ----------------- Utility methods -----------------

    private void startLaunchers() {
        if (leftLauncher != null) leftLauncher.setVelocity(LAUNCH_TARGET);
        if (rightLauncher != null) rightLauncher.setVelocity(LAUNCH_TARGET);
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0.0);
        if (rightLauncher != null) rightLauncher.setPower(0.0);
    }

    private void setPower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private boolean driveAvailable() {
        return leftFront != null && rightFront != null && leftBack != null && rightBack != null;
    }

    private void resetAllEncoders() {
        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void runToPositionAll(double power) {
        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }
    }

    private boolean anyBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }

    private void stopAndRunUsingEncoder() {
        setPower(0, 0, 0, 0);
        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

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

package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Autonomous using state machine:
 * - BACK_UP: drive backward ~48 in
 * - FIRE: spin launchers + intake for 5 sec
 * - STOP: shutdown all motors
 */
@Autonomous(name = "Backup + Fire (State Machine)", group = "RoboAvengers")
public class BackupAndShootStateMachine extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // Motion constants
    private static final double TICKS_PER_REV = 537.6;
    private static final double WHEEL_DIAM_IN = 3.78;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_IN = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAM_IN);
    private static final double MOVE_POWER = 0.5;

    // Launcher settings
    private static final double LAUNCH_TARGET = 1200;

    // State machine
    private enum AutoState { BACK_UP, FIRE, STOP, DONE }
    private AutoState state = AutoState.BACK_UP;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware setup ---
        leftFront = firstMotor("front_left_drive", "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack = firstMotor("back_left_drive", "backLeftMotor");
        rightBack = firstMotor("back_right_drive", "backRightMotor");
        intake = getMotor("intake");
        leftLauncher = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        if (leftFront != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        telemetry.addLine("READY - Backup then Fire");
        telemetry.update();

        waitForStart();
        double stateStartTime = getRuntime();

        while (opModeIsActive() && state != AutoState.DONE) {
            switch (state) {

                case BACK_UP:
                    telemetry.addLine("State: BACK_UP");
                    telemetry.update();

                    // Move 48 inches backward
                    driveBackwardInches(48, MOVE_POWER);
                    state = AutoState.FIRE;
                    stateStartTime = getRuntime();
                    break;

                case FIRE:
                    telemetry.addLine("State: FIRE (5s)");
                    telemetry.update();

                    // Start launchers + intake
                    startLaunchers();
                    if (intake != null) intake.setPower(1.0);

                    // Wait 5 seconds
                    sleep(5000);

                    stopLaunchers();
                    if (intake != null) intake.setPower(0);
                    state = AutoState.STOP;
                    stateStartTime = getRuntime();
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

            idle(); // yield to system
        }

        telemetry.addLine("AUTO COMPLETE");
        telemetry.update();
    }

    // ----------------- Movement helpers -----------------
    private void driveBackwardInches(double inches, double power) {
        if (!driveAvailable()) {
            setPower(-power, -power, -power, -power);
            sleep(1500);
            setPower(0, 0, 0, 0);
            return;
        }

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        int ticks = (int) (inches * TICKS_PER_IN);
        leftFront.setTargetPosition(-ticks);
        rightFront.setTargetPosition(-ticks);
        leftBack.setTargetPosition(-ticks);
        rightBack.setTargetPosition(-ticks);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

        while (opModeIsActive() &&
                (leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy())) {
            telemetry.addData("Moving", "%d / %d", leftFront.getCurrentPosition(), -ticks);
            telemetry.update();
            idle();
        }

        setPower(0, 0, 0, 0);
        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // ----------------- Utility methods -----------------
    private void startLaunchers() {
        if (leftLauncher != null) leftLauncher.setVelocity(LAUNCH_TARGET);
        if (rightLauncher != null) rightLauncher.setVelocity(LAUNCH_TARGET);
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
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

    private boolean driveAvailable() {
        return leftFront != null && rightFront != null && leftBack != null && rightBack != null;
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

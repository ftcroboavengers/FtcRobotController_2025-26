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

@TeleOp(name = "Decode RoboAvengers Safe Gameday", group = "RoboAvengers")
public class DecodeRoboAvengersLaunchNov8 extends LinearOpMode {

    // ----------------- Drive / Pinpoint -----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private boolean fieldCentric = true;
    private double headingOffsetRad = 0.0;

    // ----------------- Intake / Launch -----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ----------------- Launcher RPM targets -----------------
    private static final double TPR = 28.0;
    private static double rpmToTicksPerSec(double rpm) { return rpm * TPR / 60.0; }
    private static final double LAUNCH_CLOSE_RPM = 4000;
    // private static final double LAUNCH_FAR_RPM   = 6000;
    private static double LEFT_POWER_SCALE = 0.8;
    private double launcherTargetTPS = rpmToTicksPerSec(LAUNCH_CLOSE_RPM);

    // ----------------- Velocity / tolerance -----------------
    private static final int VEL_TOL = 250;
    private static final int READY_CYCLES = 2;
    private int leftReadyCount = 0, rightReadyCount = 0;

    // ----------------- Per-Side Launch States -----------------
    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    private ElapsedTime leftTimer = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();
    private static final double FEED_TIME_SECONDS = 1.0;
    private static final double FEED_POWER = 1.0;

    // ----------------- PIDF tuning -----------------
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private static double LEFT_F_GAIN  = 12.0;
    private static double RIGHT_F_GAIN = 12.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Map hardware ----
        leftFront = firstMotor("front_left_drive", "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack  = firstMotor("back_left_drive",  "backLeftMotor");
        rightBack = firstMotor("back_right_drive", "backRightMotor");

        intake = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // ---- Launcher setup ----
        if (leftLauncher != null) {
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        }

        if (rightLauncher != null) {
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);
        }

        // ---- Intake ----
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---- Drive directions ----
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

        // ---- Force PIDF ----
        if (leftLauncher != null) leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        if (rightLauncher != null) rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);

        while (opModeIsActive()) {

            // --------------- DRIVE ---------------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (gamepad1.x) fieldCentric = !fieldCentric;
            if (gamepad1.y) zeroHeading();

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);
            mecanumDrive(y, x, rx, driveScale);

            // --------------- INTAKE (manual) ---------------
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

            // --------------- CLEAR / EMERGENCY STOP ---------------
            if (gamepad2.y) {
                clearAll();
            }

            // --------------- TARGET SWITCHES ---------------
           // if (gamepad2.a) {
           //     launcherTargetTPS = rpmToTicksPerSec(LAUNCH_CLOSE_RPM);
           //     LEFT_POWER_SCALE = 0.8;
           // }
           // if (gamepad2.b) {
           //     launcherTargetTPS = rpmToTicksPerSec(LAUNCH_FAR_RPM);
           //     LEFT_POWER_SCALE = 1.0;
           // }

            // --------------- PER-SIDE SHOOT CONTROLS ---------------
            boolean unjamPressed = gamepad2.x;
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // --------------- MANUAL UNJAM or load ---------------
            if (gamepad2.x
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE) {
                startReverseUnjam();
            }

            // --------------- TELEMETRY ---------------
            if (pinpoint != null) {
                pinpoint.update();
                telemetry.addData("Heading(deg)", Math.toDegrees(getYawRad()));
                telemetry.addData("FieldCentric", fieldCentric);
            }

            telemetry.addData("Launcher Target (tps)", launcherTargetTPS);
            telemetry.addData("Left Vel (tps)", getLeftVelocity());
            telemetry.addData("Right Vel (tps)", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("Left State", leftState);
            telemetry.addData("Right State", rightState);
            telemetry.addData("Intake Power", intake != null ? intake.getPower() : 0);
            telemetry.update();
        }

        stopLaunchers();
        setPower(0, 0, 0, 0);
    }

    // ----------------- Helpers -----------------

    private void clearAll() {
        // Stop all motors immediately
        stopLaunchers();
        if (intake != null) intake.setPower(0);

        // Reset timers
        leftTimer.reset();
        rightTimer.reset();

        // Reset state machines
        leftState = LaunchState.IDLE;
        rightState = LaunchState.IDLE;

        // Reset ready counts
        leftReadyCount = 0;
        rightReadyCount = 0;

        telemetry.addLine("CLEAR ALL triggered — all systems reset.");
        telemetry.update();
    }

    private double getLeftVelocity() {
        if (leftLauncher == null) return 0;
        return -leftLauncher.getVelocity();
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

    // ----------------- Launcher Control -----------------
    private void startLeftLauncher() {
        if (leftLauncher == null) return;
        leftLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLauncher.setPower(LEFT_POWER_SCALE);
    }

    private void startRightLauncher() {
        if (rightLauncher == null) return;
        double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
        rightLauncher.setVelocity(adjusted);
    }

    private void stopLeftLauncher() { if (leftLauncher != null) leftLauncher.setPower(0); }
    private void stopRightLauncher() { if (rightLauncher != null) rightLauncher.setPower(0); }
    private void stopLaunchers() { stopLeftLauncher(); stopRightLauncher(); }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0, currentVoltage = 12.0;
        try { currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage(); }
        catch (Exception e) {}
        return targetTicksPerSec * (nominalVoltage / currentVoltage);
    }

    private boolean leftReady() {
        if (leftLauncher == null) return false;
        double leftVel = getLeftVelocity();
        boolean inTol = Math.abs(leftVel - launcherTargetTPS) <= VEL_TOL;
        leftReadyCount = inTol ? Math.min(READY_CYCLES, leftReadyCount + 1) : 0;
        return leftReadyCount >= READY_CYCLES;
    }

    private boolean rightReady() {
        if (rightLauncher == null) return false;
        boolean inTol = Math.abs(rightLauncher.getVelocity() - launcherTargetTPS) <= VEL_TOL;
        rightReadyCount = inTol ? Math.min(READY_CYCLES, rightReadyCount + 1) : 0;
        return rightReadyCount >= READY_CYCLES;
    }

    private void startReverseUnjam() {
        // Stop the intake immediately
        if (intake != null) intake.setPower(0);

        // Reset timers
        leftTimer.reset();
        rightTimer.reset();

        // Enter reverse mode
        leftState = LaunchState.REVERSE;
        rightState = LaunchState.REVERSE;

        // Start reverse motion
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
    }

    // ----------------- STATE MACHINES -----------------
    private void launchLeft(boolean shootRequested, boolean unjamRequested) {
        if (leftLauncher == null || intake == null) return;
        switch (leftState) {
            case IDLE:
                if (shootRequested) {
                    startLeftLauncher();
                    leftTimer.reset();
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (leftReady()) {
                    if (leftTimer.seconds() > 0.3) {
                        intake.setPower(FEED_POWER);
                        leftTimer.reset();
                        leftState = LaunchState.LAUNCHING;
                    }
                } else leftTimer.reset();
                break;

            case LAUNCHING:
                if (unjamRequested) {
                    intake.setPower(0);
                    stopLaunchers();
                    startReverseUnjam();
                    // Explicitly enter REVERSE state
                    leftState = LaunchState.REVERSE;
                    rightState = LaunchState.REVERSE;
                    leftTimer.reset();
                    rightTimer.reset();
                    break;
                }
                if (leftTimer.seconds() > FEED_TIME_SECONDS) {
                    intake.setPower(0);
                    leftTimer.reset();
                    leftState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                // Allow X to interrupt STOPPING and unjam
                if (unjamRequested) {
                    stopLaunchers();
                    startReverseUnjam();
                    break;
                }

                if (leftTimer.seconds() > 0.25) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (leftTimer.seconds() < 1.0) {
                    if (leftLauncher != null)  leftLauncher.setPower(-0.4);
                    if (rightLauncher != null) rightLauncher.setPower(-0.4);
                } else {
                    stopLaunchers();
                    leftTimer.reset();
                    rightTimer.reset();
                    leftReadyCount = rightReadyCount = 0;
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
                    startRightLauncher();
                    rightTimer.reset();
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (rightReady()) {
                    if (rightTimer.seconds() > 0.3) {
                        intake.setPower(FEED_POWER);
                        rightTimer.reset();
                        rightState = LaunchState.LAUNCHING;
                    }
                } else rightTimer.reset();
                break;

            case LAUNCHING:
                if (unjamRequested) {
                    intake.setPower(0);
                    stopLaunchers();
                    startReverseUnjam();
                    // Explicitly enter REVERSE state
                    leftState = LaunchState.REVERSE;
                    rightState = LaunchState.REVERSE;
                    leftTimer.reset();
                    rightTimer.reset();
                    break;
                }
                if (rightTimer.seconds() > FEED_TIME_SECONDS) {
                    intake.setPower(0);
                    rightTimer.reset();
                    rightState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (unjamRequested) {
                    stopLaunchers();
                    startReverseUnjam();
                    break;
                }

                if (rightTimer.seconds() > 0.25) {
                    stopRightLauncher();
                    rightState = LaunchState.IDLE;
                }
                break;


            case REVERSE:
                if (rightTimer.seconds() < 1.0) {
                    if (leftLauncher != null)  leftLauncher.setPower(-0.4);
                    if (rightLauncher != null) rightLauncher.setPower(-0.4);
                } else {
                    stopLaunchers();
                    leftTimer.reset();
                    rightTimer.reset();
                    leftReadyCount = rightReadyCount = 0;
                    leftState = rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ----------------- Safe Hardware Getters -----------------
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

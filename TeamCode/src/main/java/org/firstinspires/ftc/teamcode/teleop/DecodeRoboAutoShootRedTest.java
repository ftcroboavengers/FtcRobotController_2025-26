package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@TeleOp(name = "Decode Robo AutoShoot Red Test", group = "RoboAvengers")
public class DecodeRoboAutoShootRedTest extends LinearOpMode {

    // ---------------- Drive ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ---------------- Intake / Launch ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- PIDF + RPM ----------------
    private static final double TPR = 28.0; // ticks per revolution
    private static double rpmToTPS(double rpm) { return rpm * TPR / 60.0; }

    // Manual + auto both use this target
    private double launcherTargetTPS = rpmToTPS(2600); // default

    // PIDF gains (you can tweak these)
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private static final double F_GAIN = 12.0;  // your “good” value

    // Velocity tolerance for “ready”
    private static final double VEL_TOL = 200.0;
    private static final int READY_CYCLES = 2;
    private int leftReadyCount = 0;
    private int rightReadyCount = 0;

    // Timings
    private static final double FEED_TIME_SEC   = 1.0;
    private static final double STOP_DELAY_SEC  = 0.25;
    private static final double REVERSE_TIME_SEC = 1.0;
    private static final double FEED_POWER      = 1.0;

    // ---------------- Manual FSM ----------------
    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState  = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    private ElapsedTime leftTimer  = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    // ---------------- AutoShoot FSM ----------------
    private enum AutoShootState { IDLE, FIND_TAG, AIMING, SPINUP, FEEDING, STOPPING }
    private AutoShootState autoState = AutoShootState.IDLE;

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime feedTimer = new ElapsedTime();

    // AprilTag stuff
    private static final int RED_CENTER_TAG_ID = 2;
    private static final double AIM_TOL_DEG = 2.0;
    private static final int AIM_STABLE_LOOPS = 5;
    private int aimStableCount = 0;

    private double lastTagDistanceM = -1.0;
    private double lastTagBearingDeg = 0.0;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Unjam ----------------
    private boolean unjamming = false;
    private ElapsedTime unjamTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ---------------- Map hardware ----------------
        leftFront  = getMotor("front_left_drive");
        rightFront = getMotor("front_right_drive");
        leftBack   = getMotor("back_left_drive");
        rightBack  = getMotor("back_right_drive");

        intake        = getMotor("intake");
        leftLauncher  = getMotorEx("left_launcher");
        rightLauncher = getMotorEx("right_launcher");

        // Drive directions
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        // Intake direction
        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Launchers: encoders + RUN_USING_ENCODER
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // Vision
        initAprilTags();

        telemetry.addLine("TeleOp READY: AutoShoot + Manual");
        telemetry.update();
        waitForStart();

        // Apply PIDF after start
        setPIDF(leftLauncher, F_GAIN);
        setPIDF(rightLauncher, F_GAIN);

        // ================= MAIN LOOP =================
        while (opModeIsActive()) {

            // ---------- Unjam (X) ----------
            boolean unjamPressed = gamepad2.x;
            if (unjamPressed && autoState == AutoShootState.IDLE && !unjamming) {
                startUnjam();
            }
            if (unjamming) {
                updateUnjam();
            }

            // ---------- Drive ----------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05)  y = 0;
            if (Math.abs(x) < 0.05)  x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double driveScale = gamepad1.right_bumper ? 1.0 :
                    gamepad1.left_bumper  ? 0.4 : 0.7;

            if (autoState == AutoShootState.AIMING) {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                }
                double turnPower = computeAimTurnPower(lastTagBearingDeg);
                mecanumDrive(0, 0, turnPower, driveScale);
            } else if (autoState == AutoShootState.IDLE) {
                mecanumDrive(y, x, rx, driveScale);
            } else {
                mecanumDrive(0, 0, 0, 1.0);
            }

            // ---------- Intake: driver 1 triggers ----------
            if (!unjamming
                    && autoState == AutoShootState.IDLE
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE) {

                double in  = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;
                double p   = (in - out) * 0.8;

                if (Math.abs(in) < 0.02 && Math.abs(out) < 0.02) {
                    p = 0;
                }
                if (intake != null) intake.setPower(p);
            }

            // ---------- AutoShoot start: dpad_up ----------
            if (gamepad2.dpad_up
                    && autoState == AutoShootState.IDLE
                    && !unjamming
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE) {

                // Ensure everything is off before auto
                stopLaunch();
                if (intake != null) intake.setPower(0);

                startAutoShoot();
            }

            // Cancel auto with dpad_down
            if (gamepad2.dpad_down) {
                cancelAutoShoot();
            }

            // ---------- Manual shooting (bumpers) ----------
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // ---------- Manual presets (A/B/Y) ----------
            if (autoState == AutoShootState.IDLE && !unjamming) {

                // A → short (2300 rpm)
                if (gamepad2.a) {
                    double rpm = 2300;
                    launcherTargetTPS = rpmToTPS(rpm);
                    setPIDF(leftLauncher, F_GAIN);
                    setPIDF(rightLauncher, F_GAIN);
                    telemetry.addLine("Preset: SHORT (2300 rpm, F=12)");
                }

                // B → mid (2400 rpm)
                if (gamepad2.b) {
                    double rpm = 2400;
                    launcherTargetTPS = rpmToTPS(rpm);
                    setPIDF(leftLauncher, F_GAIN);
                    setPIDF(rightLauncher, F_GAIN);
                    telemetry.addLine("Preset: MID (2400 rpm, F=12)");
                }

                // Y → long (2600 rpm)
                if (gamepad2.y) {
                    double rpm = 2600;
                    launcherTargetTPS = rpmToTPS(rpm);
                    setPIDF(leftLauncher, F_GAIN);
                    setPIDF(rightLauncher, F_GAIN);
                    telemetry.addLine("Preset: LONG (2600 rpm, F=12)");
                }
            }

            // ---------- Manual hold-spin (triggers) ----------
            if (autoState == AutoShootState.IDLE && !unjamming) {

                // Left launcher
                if (leftLauncher != null && leftState == LaunchState.IDLE) {
                    if (gamepad2.left_trigger > 0.2) {
                        leftLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));
                    } else if (!gamepad2.left_bumper) {
                        leftLauncher.setPower(0);
                    }
                }

                // Right launcher
                if (rightLauncher != null && rightState == LaunchState.IDLE) {
                    if (gamepad2.right_trigger > 0.2) {
                        rightLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));
                    } else if (!gamepad2.right_bumper) {
                        rightLauncher.setPower(0);
                    }
                }
            }

            // ---------- Manual feed (right stick button) ----------
            if (autoState == AutoShootState.IDLE
                    && leftState == LaunchState.IDLE
                    && rightState == LaunchState.IDLE
                    && !unjamming) {

                if (gamepad2.right_stick_button) {
                    if (intake != null) intake.setPower(FEED_POWER);
                } else if (Math.abs(gamepad1.left_trigger) < 0.01
                        && Math.abs(gamepad1.right_trigger) < 0.01) {
                    if (intake != null) intake.setPower(0);
                }
            }

            // ---------- Panic stop (back) ----------
            if (gamepad2.back) {
                stopLaunch();
                if (intake != null) intake.setPower(0);
                leftState  = LaunchState.IDLE;
                rightState = LaunchState.IDLE;
                unjamming  = false;
                autoState  = AutoShootState.IDLE;
            }

            // ---------- AutoShoot FSM ----------
            updateAutoShoot();

            // ---------- Telemetry ----------
            telemetry.addData("AutoState", autoState);
            telemetry.addData("LeftState", leftState);
            telemetry.addData("RightState", rightState);
            telemetry.addData("TargetRPM", launcherTargetTPS * 60.0 / TPR);
            telemetry.addData("LeftVel",  leftLauncher  != null ? leftLauncher.getVelocity()  : 0);
            telemetry.addData("RightVel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("TagRange(m)",   lastTagDistanceM);
            telemetry.addData("TagBearing(deg)", lastTagBearingDeg);
            telemetry.addData("Unjamming", unjamming);
            telemetry.update();
        }

        stopAll();
    }

    // ============================================================
    // AUTO SHOOT
    // ============================================================

    private void startAutoShoot() {
        autoState = AutoShootState.FIND_TAG;
        autoTimer.reset();
        aimStableCount = 0;
    }

    private void cancelAutoShoot() {
        autoState = AutoShootState.IDLE;
        if (intake != null) intake.setPower(0);
        stopLaunch();
    }

    private void updateAutoShoot() {
        switch (autoState) {

            case IDLE:
                return;

            case FIND_TAG: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    autoState = AutoShootState.AIMING;
                } else if (autoTimer.seconds() > 2.0) {
                    cancelAutoShoot();
                }
                return;
            }

            case AIMING: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                }

                if (Math.abs(lastTagBearingDeg) <= AIM_TOL_DEG) {
                    aimStableCount++;
                } else {
                    aimStableCount = 0;
                }

                if (aimStableCount >= AIM_STABLE_LOOPS) {
                    double rpm = computeAutoRpm(lastTagDistanceM);
                    launcherTargetTPS = rpmToTPS(rpm);
                    setPIDF(leftLauncher, F_GAIN);
                    setPIDF(rightLauncher, F_GAIN);
                    startLaunchers();
                    autoTimer.reset();
                    autoState = AutoShootState.SPINUP;
                }
                return;
            }

            case SPINUP: {
                boolean ready = leftReady() && rightReady();
                if (ready && autoTimer.seconds() > 0.3) {
                    if (intake != null) intake.setPower(FEED_POWER);
                    feedTimer.reset();
                    autoState = AutoShootState.FEEDING;
                } else if (autoTimer.seconds() > 3.0) {
                    cancelAutoShoot();
                }
                return;
            }

            case FEEDING: {
                if (feedTimer.seconds() > FEED_TIME_SEC) {
                    if (intake != null) intake.setPower(0);
                    autoTimer.reset();
                    autoState = AutoShootState.STOPPING;
                }
                return;
            }

            case STOPPING: {
                if (autoTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunch();
                    autoState = AutoShootState.IDLE;
                }
            }
        }
    }

    private double computeAutoRpm(double distanceM) {
        if (distanceM <= 0) return 2400;  // fallback
        double a = 600.0;
        double b = 2050.0;
        double rpm = a * distanceM + b;
        return Math.max(2400, Math.min(3000, rpm));
    }

    // ============================================================
    // MANUAL FSM LAUNCH
    // ============================================================

    private void launchLeft(boolean shootRequested, boolean unjamPressed) {
        if (leftLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;
        if (unjamming) return;

        switch (leftState) {
            case IDLE:
                if (shootRequested) {
                    startLeftLauncher();
                    leftTimer.reset();
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (leftReady() && leftTimer.seconds() > 0.3) {
                    intake.setPower(FEED_POWER);
                    leftTimer.reset();
                    leftState = LaunchState.LAUNCHING;
                } else if (!leftReady()) {
                    leftTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamPressed) { startUnjam(); break; }
                if (leftTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    leftTimer.reset();
                    leftState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (leftTimer.seconds() > STOP_DELAY_SEC) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                // handled by unjam timer
                break;
        }
    }

    private void launchRight(boolean shootRequested, boolean unjamPressed) {
        if (rightLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;
        if (unjamming) return;

        switch (rightState) {
            case IDLE:
                if (shootRequested) {
                    startRightLauncher();
                    rightTimer.reset();
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (rightReady() && rightTimer.seconds() > 0.3) {
                    intake.setPower(FEED_POWER);
                    rightTimer.reset();
                    rightState = LaunchState.LAUNCHING;
                } else if (!rightReady()) {
                    rightTimer.reset();
                }
                break;

            case LAUNCHING:
                if (unjamPressed) { startUnjam(); break; }
                if (rightTimer.seconds() > FEED_TIME_SEC) {
                    intake.setPower(0);
                    rightTimer.reset();
                    rightState = LaunchState.STOPPING;
                }
                break;

            case STOPPING:
                if (rightTimer.seconds() > STOP_DELAY_SEC) {
                    stopRightLauncher();
                    rightState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                // handled by unjam timer
                break;
        }
    }

    // ============================================================
    // LAUNCHER HELPERS
    // ============================================================

    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setZeroPowerBehavior(BRAKE);
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPIDF(DcMotorEx m, double f) {
        if (m != null) {
            m.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, f);
        }
    }

    private double getVoltageCompensatedVelocity(double targetTPS) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;

        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception ignored) {}

        if (currentVoltage <= 0) currentVoltage = 12.0;

        return targetTPS * (nominalVoltage / currentVoltage);
    }

    private boolean leftReady() {
        if (leftLauncher == null) return false;
        double v = leftLauncher.getVelocity();
        boolean inTol = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        leftReadyCount = inTol ? Math.min(READY_CYCLES, leftReadyCount + 1) : 0;
        return leftReadyCount >= READY_CYCLES;
    }

    private boolean rightReady() {
        if (rightLauncher == null) return false;
        double v = rightLauncher.getVelocity();
        boolean inTol = Math.abs(v - launcherTargetTPS) <= VEL_TOL;
        rightReadyCount = inTol ? Math.min(READY_CYCLES, rightReadyCount + 1) : 0;
        return rightReadyCount >= READY_CYCLES;
    }

    private void startLaunchers() {
        startLeftLauncher();
        startRightLauncher();
        leftReadyCount = 0;
        rightReadyCount = 0;
    }

    private void stopLaunch() {
        stopLeftLauncher();
        stopRightLauncher();
    }

    private void startLeftLauncher() {
        if (leftLauncher != null) {
            leftLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));
        }
    }

    private void startRightLauncher() {
        if (rightLauncher != null) {
            rightLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));
        }
    }

    private void stopLeftLauncher() {
        if (leftLauncher != null) leftLauncher.setPower(0);
    }

    private void stopRightLauncher() {
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    // ============================================================
    // UNJAM (X) – LAUNCHERS ONLY
    // ============================================================

    private void startUnjam() {
        unjamming = true;
        unjamTimer.reset();

        // Reverse only the launchers; keep intake off
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        if (intake != null)        intake.setPower(0);

        leftState  = LaunchState.REVERSE;
        rightState = LaunchState.REVERSE;
        leftTimer.reset();
        rightTimer.reset();
    }

    private void updateUnjam() {
        if (unjamTimer.seconds() > REVERSE_TIME_SEC) {
            unjamming = false;
            stopLaunch();
            if (intake != null) intake.setPower(0);
            leftState  = LaunchState.IDLE;
            rightState = LaunchState.IDLE;
        }
    }

    // ============================================================
    // APRILTAG VISION
    // ============================================================

    private void initAprilTags() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .enableLiveView(true)
                .build();
    }

    private AprilTagDetection getBestRedTag() {
        if (tagProcessor == null) return null;

        for (AprilTagDetection tag : tagProcessor.getDetections()) {
            if (tag.metadata != null && tag.ftcPose != null) {
                int id = tag.metadata.id;
                if (id == RED_CENTER_TAG_ID || id == 1 || id == 3) {
                    return tag;
                }
            }
        }
        return null;
    }

    private double computeAimTurnPower(double bearingDeg) {
        double kP = 0.02;
        double raw = -kP * bearingDeg;
        double min = 0.12, max = 0.4;

        if (Math.abs(raw) < min && Math.abs(bearingDeg) > AIM_TOL_DEG) {
            raw = Math.signum(raw) * min;
        }
        if (Math.abs(raw) > max) {
            raw = Math.signum(raw) * max;
        }
        return raw;
    }

    // ============================================================
    // DRIVE HELPERS
    // ============================================================

    private void mecanumDrive(double y, double x, double rx, double scale) {
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denom * scale;
        double bl = (y - x + rx) / denom * scale;
        double fr = (y - x - rx) / denom * scale;
        double br = (y + x - rx) / denom * scale;

        setPower(fl, fr, bl, br);
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

    private void stopAll() {
        stopLaunch();
        if (intake != null) intake.setPower(0);
        setPower(0, 0, 0, 0);
    }

    // ============================================================
    // SAFE HARDWARE GETTERS
    // ============================================================

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }

    private DcMotorEx getMotorEx(String name) {
        try { return hardwareMap.get(DcMotorEx.class, name); }
        catch (Exception e) { return null; }
    }
}

package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Decode Robo AutoShoot Red", group = "RoboAvengers")
public class DecodeRoboAutoShootRed extends LinearOpMode {

    // ---------------- Drive ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ---------------- Intake / Launch ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- PIDF STUFF ----------------
    private static final double TPR = 28.0; // ticks per revolution (Gobilda encoder)
    private double launcherTargetTPS = rpmToTicksPerSec(4000.0);

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TPR / 60.0;
    }

    // PIDF Gains
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private static final double LEFT_F_GAIN  = 12.0;
    private static final double RIGHT_F_GAIN = 12.0;

    // Launcher “ready” tolerance
    private static final double VEL_TOL = 250;
    private static final int READY_CYCLES = 2;
    private int leftReadyCount = 0;
    private int rightReadyCount = 0;

    // Timings
    private static final double FEED_TIME_SEC    = 1.0;
    private static final double STOP_DELAY_SEC   = 0.25;
    private static final double REVERSE_TIME_SEC = 1.0;
    private static final double FEED_POWER       = 1.0;

    // ---------------- Manual Launcher States ----------------
    private enum LaunchState { IDLE, SPIN_UP, LAUNCHING, STOPPING, REVERSE }
    private LaunchState leftState  = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;
    private ElapsedTime leftTimer  = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    // ---------------- Auto-Shoot State Machine ----------------
    private enum AutoShootState { IDLE, FIND_TAG, AIMING, SPINUP, FEEDING, STOPPING }
    private AutoShootState autoState = AutoShootState.IDLE;

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime feedTimer = new ElapsedTime();

    // Auto-aim values
    private static final int RED_CENTER_TAG_ID = 2;
    private static final double AIM_TOL_DEG = 2.0;
    private static final int AIM_STABLE_LOOPS = 5;
    private int aimStableCount = 0;

    private double lastTagDistanceM = -1;
    private double lastTagBearingDeg = 0;

    // Auto-shoot timeouts
    private static final double MIN_SPINUP_SEC   = 0.3;
    private static final double SPINUP_TIMEOUT_S = 3.0;
    private static final double FIND_TIMEOUT_S   = 2.0;

    // ---------------- Vision System ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    // ---------------- Unjam ----------------
    private boolean unjamming = false;
    private ElapsedTime unjamTimer = new ElapsedTime();

    private static final double TAG_MIN_CONF = 0.6;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- Map hardware ----
        leftFront  = getMotor("front_left_drive");
        rightFront = getMotor("front_right_drive");
        leftBack   = getMotor("back_left_drive");
        rightBack  = getMotor("back_right_drive");

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

        // ---- Launchers ----
        initLauncher(leftLauncher, true);
        initLauncher(rightLauncher, false);

        // ---- Vision ----
        initAprilTags();

        telemetry.addLine("TeleOp READY: AutoShoot + Manual Shoot.");
        telemetry.update();
        waitForStart();

        // Reapply PIDF after start
        setPIDF(leftLauncher, LEFT_F_GAIN);
        setPIDF(rightLauncher, RIGHT_F_GAIN);

        while (opModeIsActive()) {

            // Manual unjam override
            boolean unjamPressed = gamepad2.x;
            if (unjamPressed && autoState == AutoShootState.IDLE) {
                startReverseUnjam();
            }
            if (unjamming) updateUnjam();

            // --------------------------------
            // DRIVE CONTROL
            // --------------------------------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);

            // Auto aim blocks driving
            if (autoState == AutoShootState.AIMING) {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    lastTagDistanceM = tag.ftcPose.range;
                }
                double autoRx = computeAimTurnPower(lastTagBearingDeg);
                mecanumDrive(0, 0, autoRx, driveScale);

            } else if (autoState == AutoShootState.IDLE) {
                // driver normal
                mecanumDrive(y, x, rx, driveScale);

            } else {
                // other auto states → freeze
                mecanumDrive(0, 0, 0, 1.0);
            }

            // --------------------------------
            // INTAKE (manual only)
            // --------------------------------
            if (!unjamming && autoState == AutoShootState.IDLE) {
                double in = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;
                double p = (in - out) * 1.0;
                if (intake != null) intake.setPower(p);
            }

            // --------------------------------
            // AUTO-SHOOT START (one button)
            // --------------------------------
            if (autoState == AutoShootState.IDLE &&
                    gamepad2.dpad_up && !unjamming) {
                startAutoShoot();
            }

            // Manual cancel
            if (gamepad2.dpad_down) {
                cancelAutoShoot();
            }

            // --------------------------------
            // MANUAL SHOOTING (left & right)
            // --------------------------------
            launchLeft(gamepad2.left_bumper, unjamPressed);
            launchRight(gamepad2.right_bumper, unjamPressed);

            // --------------------------------
            // MANUAL SHOOTING PRESETS (gamepad2)
            // --------------------------------
            if (autoState == AutoShootState.IDLE && !unjamming) {

                // A → CLOSE TRIANGLE
                if (gamepad2.a) {
                    double rpm = 2600;
                    launcherTargetTPS = rpmToTicksPerSec(rpm);
                    applyDynamicF(rpm);  // sets F = 12
                    telemetry.addLine("Preset: CLOSE (2600 rpm, F=12)");
                }

                // Y → FAR TRIANGLE
                if (gamepad2.y) {
                    double rpm = 2800;
                    launcherTargetTPS = rpmToTicksPerSec(rpm);
                    applyDynamicF(rpm);  // sets F = 17
                    telemetry.addLine("Preset: FAR (2800 rpm, F=17)");
                }

                // B → SMALL SIDE TRIANGLE
                if (gamepad2.b) {
                    // (ignore B used for canceling auto-shoot)
                    double rpm = 2500;
                    launcherTargetTPS = rpmToTicksPerSec(rpm);
                    applyDynamicF(rpm);  // sets F = 11
                    telemetry.addLine("Preset: SIDE (2500 rpm, F=11)");
                }
            }


            // --------------------------------
            // AUTO-SHOOTING STATE MACHINE
            // --------------------------------
            updateAutoShoot();

            // --------------------------------
            // TELEMETRY
            // --------------------------------
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Manual Left State", leftState);
            telemetry.addData("Manual Right State", rightState);
            telemetry.addData("Tag Dist (m)", lastTagDistanceM);
            telemetry.addData("Tag Bearing (deg)", lastTagBearingDeg);
            telemetry.addData("Target TPS", launcherTargetTPS);
            telemetry.addData("Left Vel", leftLauncher != null ? leftLauncher.getVelocity() : 0);
            telemetry.addData("Right Vel", rightLauncher != null ? rightLauncher.getVelocity() : 0);
            telemetry.addData("Unjamming", unjamming);
            telemetry.update();
        }

        stopAll();
    }

    // ---------------- Launcher Init ----------------
    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setZeroPowerBehavior(BRAKE);
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPIDF(DcMotorEx m, double f) {
        if (m != null) m.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, f);
    }

    // ---------------- AutoShoot: Start / Cancel / Update ----------------
    private void startAutoShoot() {
        autoState = AutoShootState.FIND_TAG;
        autoTimer.reset();
        aimStableCount = 0;
        launcherTargetTPS = rpmToTicksPerSec(4000);
    }

    private void cancelAutoShoot() {
        autoState = AutoShootState.IDLE;
        aimStableCount = 0;
        if (intake != null) intake.setPower(0);
        stopLaunchers();
    }

    private void updateAutoShoot() {

        switch (autoState) {

            case IDLE:
                break;

            case FIND_TAG: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    autoState = AutoShootState.AIMING;
                } else if (autoTimer.seconds() > FIND_TIMEOUT_S) {
                    cancelAutoShoot();
                }
                break;
            }

            case AIMING: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null) {
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

                    // Apply your F-gain curve
                    applyDynamicF(rpm);

                    // Convert RPM → ticks
                    launcherTargetTPS = rpmToTicksPerSec(rpm);

                    // Start spinning both wheels
                    startLaunchers();

                    autoTimer.reset();
                    autoState = AutoShootState.SPINUP;
                }
                break;
            }

            case SPINUP: {
                boolean ready = leftReady() && rightReady();
                if (ready && autoTimer.seconds() > MIN_SPINUP_SEC) {
                    if (intake != null) intake.setPower(FEED_POWER);
                    feedTimer.reset();
                    autoState = AutoShootState.FEEDING;
                } else if (autoTimer.seconds() > SPINUP_TIMEOUT_S) {
                    cancelAutoShoot();
                }
                break;
            }

            case FEEDING: {
                if (feedTimer.seconds() > FEED_TIME_SEC) {
                    if (intake != null) intake.setPower(0);
                    autoTimer.reset();
                    autoState = AutoShootState.STOPPING;
                }
                break;
            }

            case STOPPING: {
                if (autoTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunchers();
                    autoState = AutoShootState.IDLE;
                }
                break;
            }
        }
    }

    // ---------------- Auto RPM Model ----------------
    private double computeAutoRpm(double distanceM) {
        if (distanceM <= 0) return 2600; // fallback

        double a = 600;     // rpm per meter
        double b = 2050;    // base rpm

        double rpm = a * distanceM + b;

        // Clamp for safety
        return Math.max(2400, Math.min(3000, rpm));
    }

    // Dynamically adjusts F-gain to match RPM
    private void applyDynamicF(double rpm) {
        double f = 0.02 * rpm - 39;

        // safety clamp
        f = Math.max(8, Math.min(20, f));

        setPIDF(leftLauncher, f);
        setPIDF(rightLauncher, f);
    }

    // ---------------- Auto Aim Turn Power ----------------
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

    // ---------------- Manual Shooting ----------------
    private void launchLeft(boolean shootRequested, boolean unjamRequested) {
        if (leftLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;

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
                if (unjamRequested) {
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
                if (leftTimer.seconds() > STOP_DELAY_SEC) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;

            case REVERSE:
                if (leftTimer.seconds() > REVERSE_TIME_SEC) {
                    stopLeftLauncher();
                    leftState = LaunchState.IDLE;
                }
                break;
        }
    }

    private void launchRight(boolean shootRequested, boolean unjamRequested) {
        if (rightLauncher == null || intake == null) return;
        if (autoState != AutoShootState.IDLE) return;

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
                if (unjamRequested) {
                    startReverseUnjam();
                    break;
                }
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
                if (rightTimer.seconds() > REVERSE_TIME_SEC) {
                    stopRightLauncher();
                    rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ---------------- Manual Launcher Control ----------------
    private void startLeftLauncher() {
        double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
        leftLauncher.setVelocity(adjusted);
    }

    private void startRightLauncher() {
        double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
        rightLauncher.setVelocity(adjusted);
    }

    private void stopLeftLauncher() {
        leftLauncher.setPower(0);
    }

    private void stopRightLauncher() {
        rightLauncher.setPower(0);
    }

    // ---------------- Unjam ----------------
    private void startReverseUnjam() {
        unjamming = true;
        unjamTimer.reset();
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
        if (intake != null) intake.setPower(-1.0);
        leftState = LaunchState.REVERSE;
        rightState = LaunchState.REVERSE;
    }

    private void updateUnjam() {
        if (unjamTimer.seconds() > REVERSE_TIME_SEC) {
            unjamming = false;
            stopLaunchers();
            if (intake != null) intake.setPower(0);
        }
    }

    // ---------------- Vision ----------------
    private void initAprilTags() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawAxes(true)
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
                if (id == RED_CENTER_TAG_ID && tag.ftcPose.range > 0) return tag;
                if ((id == 1 || id == 3) && tag.ftcPose.range > 0) return tag;
            }
        }
        return null;
    }

    // ---------------- Launcher Helpers ----------------
    private void startLaunchers() {
        if (leftLauncher != null)
            leftLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));
        if (rightLauncher != null)
            rightLauncher.setVelocity(getVoltageCompensatedVelocity(launcherTargetTPS));

        leftReadyCount = 0;
        rightReadyCount = 0;
    }

    private void stopLaunchers() {
        if (leftLauncher != null) leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;

        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception ignored) {}

        if (currentVoltage <= 0) currentVoltage = 12.0;

        return targetTicksPerSec * (nominalVoltage / currentVoltage);
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

    // ---------------- Drive Helpers ----------------
    private void mecanumDrive(double y, double x, double rx, double scale) {
        if (leftFront == null) return;

        double rotX = x;
        double rotY = y;

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
        for (DcMotor m : motors) {
            if (m != null) m.setZeroPowerBehavior(BRAKE);
        }
    }

    private void stopAll() {
        stopLaunchers();
        if (intake != null) intake.setPower(0);
        setPower(0, 0, 0, 0);
    }

    // ---------------- Hardware Getters ----------------
    private DcMotor getMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }

    private DcMotorEx getMotorEx(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}

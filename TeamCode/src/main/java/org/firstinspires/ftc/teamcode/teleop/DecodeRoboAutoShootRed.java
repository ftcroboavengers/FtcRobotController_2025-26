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

    // ---------------- Launcher RPM / PIDF ----------------
    private static final double TPR = 28.0; // ticks per revolution (GoBilda encoder, adjust if geared)

    private static double rpmToTicksPerSec(double rpm) {
        return rpm * TPR / 60.0;
    }

    // Default close/far ranges (only used as clamps)
    private static final double MIN_RPM = 3500.0;
    private static final double MAX_RPM = 6000.0;

    // Dynamic target velocity (ticks/sec)
    private double launcherTargetTPS = rpmToTicksPerSec(4000.0);

    // Velocity tolerance for "ready"
    private static final int VEL_TOL = 250;      // ticks/sec tolerance
    private static final int READY_CYCLES = 2;   // consecutive loops in tolerance
    private int leftReadyCount  = 0;
    private int rightReadyCount = 0;

    // PIDF gains (tune F per motor as needed)
    private static final double P_GAIN = 25.0;
    private static final double I_GAIN = 0.0;
    private static final double D_GAIN = 5.0;
    private static final double LEFT_F_GAIN  = 12.0;
    private static final double RIGHT_F_GAIN = 12.0;

    // ---------------- Timings ----------------
    private static final double FEED_TIME_SEC    = 1.0;
    private static final double STOP_DELAY_SEC   = 0.25;
    private static final double REVERSE_TIME_SEC = 1.0;
    private static final double FEED_POWER       = 1.0;

    // ---------------- Auto-Shoot State Machine ----------------
    private enum AutoShootState { IDLE, FIND_TAG, AIMING, SPINUP, FEEDING, STOPPING }
    private AutoShootState autoState = AutoShootState.IDLE;

    private ElapsedTime autoTimer = new ElapsedTime();
    private ElapsedTime feedTimer = new ElapsedTime();
    private ElapsedTime unjamTimer = new ElapsedTime();

    private boolean prevAutoButton = false;

    // Auto-aim stability
    private static final double AIM_TOL_DEG = 2.0;      // acceptable bearing error
    private static final int AIM_STABLE_LOOPS = 5;
    private int aimStableCount = 0;

    // Spin-up timing
    private static final double MIN_SPINUP_SEC   = 0.3;
    private static final double SPINUP_TIMEOUT_S = 3.0;
    private static final double FIND_TIMEOUT_S   = 2.0;

    // ---------------- AprilTag Vision ----------------
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;

    private static final int RED_CENTER_TAG_ID = 2; // main target
    private static final double TAG_MIN_CONF   = 0.6;

    private double lastTagDistanceM = -1;
    private double lastTagBearingDeg = 0;

    // ---------------- Unjam flag ----------------
    private boolean unjamming = false;

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

        // ---- Launchers (encoder + PIDF) ----
        if (leftLauncher != null) {
            leftLauncher.setZeroPowerBehavior(BRAKE);
            leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
            leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        }
        if (rightLauncher != null) {
            rightLauncher.setZeroPowerBehavior(BRAKE);
            rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);
        }

        // ---- Vision ----
        initAprilTags();

        telemetry.addLine("TeleOp READY: AutoShoot Red Enabled");
        telemetry.update();
        waitForStart();

        // Re-apply PIDF after start in case FTC SDK touches it
        if (leftLauncher != null) {
            leftLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, LEFT_F_GAIN);
        }
        if (rightLauncher != null) {
            rightLauncher.setVelocityPIDFCoefficients(P_GAIN, I_GAIN, D_GAIN, RIGHT_F_GAIN);
        }

        while (opModeIsActive()) {

            // -------- Read controls --------
            boolean autoButton = gamepad2.right_stick_button; // one-button AutoShoot
            boolean cancelButton = gamepad2.b;
            boolean unjamButton = gamepad2.x;

            // Edge detection for starting AutoShoot
            if (autoButton && !prevAutoButton && autoState == AutoShootState.IDLE && !unjamming) {
                startAutoShoot();
            }
            prevAutoButton = autoButton;

            // Cancel AutoShoot
            if (cancelButton) {
                cancelAutoShoot();
            }

            // Unjam (manual)
            if (unjamButton && !unjamming && autoState == AutoShootState.IDLE) {
                startUnjam();
            }
            if (unjamming) {
                updateUnjam();
            }

            // -------- DRIVE --------
            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // Apply deadzone
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double driveScale = gamepad1.right_bumper ? 1.0 : (gamepad1.left_bumper ? 0.4 : 0.7);

            // During AIMING, override rotation with auto-aim; block translation
            if (autoState == AutoShootState.AIMING) {
                // Use vision-based bearing to set rx
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null) {
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    lastTagDistanceM = tag.ftcPose.range;
                }
                double autoRx = computeAimTurnPower(lastTagBearingDeg);
                mecanumDrive(0, 0, autoRx, driveScale);
            } else if (autoState == AutoShootState.IDLE) {
                // Normal driver control
                mecanumDrive(y, x, rx, driveScale);
            } else {
                // Other auto states: keep robot still
                mecanumDrive(0, 0, 0, driveScale);
            }

            // -------- INTAKE (manual only when not auto-shooting or unjamming) --------
            if (intake != null && autoState == AutoShootState.IDLE && !unjamming) {
                double in = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;

                double p = 0;
                double scale = 0.7;
                if (in > 0.01 || out > 0.01) {
                    p = (in - out) * scale;
                }
                intake.setPower(p);
            }

            // -------- AutoShoot State Machine --------
            updateAutoShoot();

            // -------- Telemetry --------
            telemetry.addData("Auto State", autoState);
            telemetry.addData("Last Tag Dist (m)", lastTagDistanceM);
            telemetry.addData("Last Tag Bearing (deg)", lastTagBearingDeg);
            telemetry.addData("Target (tps)", launcherTargetTPS);
            telemetry.addData("Left Vel (tps)", leftLauncher != null ? leftLauncher.getVelocity() : 0.0);
            telemetry.addData("Right Vel (tps)", rightLauncher != null ? rightLauncher.getVelocity() : 0.0);
            telemetry.addData("Left Ready", leftReady());
            telemetry.addData("Right Ready", rightReady());
            telemetry.addData("Unjamming", unjamming);
            telemetry.update();
        }

        stopAll();
    }

    // ---------------- AutoShoot helpers ----------------

    private void startAutoShoot() {
        autoState = AutoShootState.FIND_TAG;
        autoTimer.reset();
        aimStableCount = 0;
        telemetry.addLine("AutoShoot: FIND_TAG");
        telemetry.update();
    }

    private void cancelAutoShoot() {
        autoState = AutoShootState.IDLE;
        aimStableCount = 0;
        if (intake != null) intake.setPower(0);
        stopLaunchers();
        telemetry.addLine("AutoShoot: CANCELLED");
        telemetry.update();
    }

    private void updateAutoShoot() {
        switch (autoState) {
            case IDLE:
                // nothing
                break;

            case FIND_TAG: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null && tag.ftcPose.range > 0) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                    aimStableCount = 0;
                    autoState = AutoShootState.AIMING;
                } else if (autoTimer.seconds() > FIND_TIMEOUT_S) {
                    telemetry.addLine("AutoShoot: No tag found, aborting.");
                    telemetry.update();
                    cancelAutoShoot();
                }
                break;
            }

            case AIMING: {
                AprilTagDetection tag = getBestRedTag();
                if (tag != null && tag.ftcPose != null && tag.ftcPose.range > 0) {
                    lastTagDistanceM = tag.ftcPose.range;
                    lastTagBearingDeg = tag.ftcPose.bearing;
                }

                if (Math.abs(lastTagBearingDeg) <= AIM_TOL_DEG) {
                    aimStableCount = Math.min(AIM_STABLE_LOOPS, aimStableCount + 1);
                } else {
                    aimStableCount = 0;
                }

                if (aimStableCount >= AIM_STABLE_LOOPS) {
                    // Aim locked: compute RPM from distance and spin up
                    double rpm = computeAutoRpm(lastTagDistanceM);
                    launcherTargetTPS = rpmToTicksPerSec(rpm);
                    startLaunchers();
                    autoTimer.reset();
                    autoState = AutoShootState.SPINUP;
                    telemetry.addData("AutoShoot: SPINUP, RPM", rpm);
                    telemetry.update();
                }
                break;
            }

            case SPINUP: {
                boolean ready = leftReady() && rightReady();
                if (ready && autoTimer.seconds() > MIN_SPINUP_SEC) {
                    if (intake != null) intake.setPower(FEED_POWER);
                    feedTimer.reset();
                    autoState = AutoShootState.FEEDING;
                    telemetry.addLine("AutoShoot: FEEDING");
                    telemetry.update();
                } else if (autoTimer.seconds() > SPINUP_TIMEOUT_S) {
                    telemetry.addLine("AutoShoot: Spin-up timeout, aborting.");
                    telemetry.update();
                    cancelAutoShoot();
                }
                break;
            }

            case FEEDING: {
                if (feedTimer.seconds() > FEED_TIME_SEC) {
                    if (intake != null) intake.setPower(0);
                    autoTimer.reset();
                    autoState = AutoShootState.STOPPING;
                    telemetry.addLine("AutoShoot: STOPPING");
                    telemetry.update();
                }
                break;
            }

            case STOPPING: {
                if (autoTimer.seconds() > STOP_DELAY_SEC) {
                    stopLaunchers();
                    autoState = AutoShootState.IDLE;
                    telemetry.addLine("AutoShoot: COMPLETE");
                    telemetry.update();
                }
                break;
            }
        }
    }

    // Compute RPM from tag distance (meters) — tune these numbers on the field
    private double computeAutoRpm(double distanceM) {
        if (distanceM <= 0) {
            return 4000; // fallback
        }

        // Example linear model: adjust a and b from testing
        double a = 2000;   // rpm per meter
        double b = 2000;   // base rpm
        double rpm = a * distanceM + b;

        // Clamp between min and max
        rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
        return rpm;
    }

    // Compute turn power from bearing (deg)
    private double computeAimTurnPower(double bearingDeg) {
        // Positive bearing = tag to the left
        double kP = 0.02; // turn gain
        double raw = -kP * bearingDeg; // negative to turn toward the tag

        double min = 0.12;
        double max = 0.4;

        if (Math.abs(raw) < min && Math.abs(bearingDeg) > AIM_TOL_DEG) {
            raw = Math.signum(raw) * min;
        }
        if (Math.abs(raw) > max) {
            raw = Math.signum(raw) * max;
        }
        return raw;
    }

    // ---------------- Unjam helpers ----------------

    private void startUnjam() {
        unjamming = true;
        unjamTimer.reset();
        if (intake != null) intake.setPower(0);
        if (leftLauncher != null)  leftLauncher.setPower(-0.4);
        if (rightLauncher != null) rightLauncher.setPower(-0.4);
    }

    private void updateUnjam() {
        if (unjamTimer.seconds() > REVERSE_TIME_SEC) {
            if (leftLauncher != null)  leftLauncher.setPower(0);
            if (rightLauncher != null) rightLauncher.setPower(0);
            unjamming = false;
        }
    }

    // ---------------- Vision helpers ----------------

    private void initAprilTags() {
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
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

        AprilTagDetection best = null;

        for (AprilTagDetection tag : tagProcessor.getDetections()) {
            if (tag.metadata == null || tag.ftcPose == null) continue;
            int id = tag.metadata.id;

            // Red backboard tags are usually 1, 2, 3; prefer center (2)
            if (id == RED_CENTER_TAG_ID) {
                if (tag.ftcPose.range > 0) {
                    return tag;
                }
            } else if (id == 1 || id == 3) {
                if (tag.ftcPose.range > 0 && best == null) {
                    best = tag;
                }
            }
        }
        return best;
    }

    // ---------------- Launcher helpers ----------------

    private void startLaunchers() {
        if (leftLauncher != null) {
            double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
            leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLauncher.setVelocity(adjusted);
        }
        if (rightLauncher != null) {
            double adjusted = getVoltageCompensatedVelocity(launcherTargetTPS);
            rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLauncher.setVelocity(adjusted);
        }
        leftReadyCount = 0;
        rightReadyCount = 0;
    }

    private void stopLaunchers() {
        if (leftLauncher != null) {
            leftLauncher.setPower(0);
        }
        if (rightLauncher != null) {
            rightLauncher.setPower(0);
        }
    }

    private double getVoltageCompensatedVelocity(double targetTicksPerSec) {
        double nominalVoltage = 13.0;
        double currentVoltage = 12.0;
        try {
            currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        } catch (Exception e) {
            // ignore, fallback to 12V
        }
        if (currentVoltage <= 0.0) {
            currentVoltage = 12.0;
        }
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

    // ---------------- Drive helpers ----------------

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
            if (m != null) {
                m.setZeroPowerBehavior(BRAKE);
            }
        }
    }

    private void stopAll() {
        stopLaunchers();
        if (intake != null) intake.setPower(0);
        setPower(0, 0, 0, 0);
    }

    // ---------------- Safe Getters ----------------
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

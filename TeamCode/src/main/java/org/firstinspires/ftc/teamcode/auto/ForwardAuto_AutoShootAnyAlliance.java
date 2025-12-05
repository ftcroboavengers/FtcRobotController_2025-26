package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Forward + Shoot", group = "RoboAvengers")
public class ForwardAuto_AutoShootAnyAlliance extends LinearOpMode {

    // ---------------- Drive Motors ----------------
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ---------------- Intake + Launcher ----------------
    private DcMotor intake;
    private DcMotorEx leftLauncher, rightLauncher;

    // ---------------- Odometry ----------------
    private GoBildaPinpointDriver pinpoint;

    // ---------------- Motion Tuning ----------------
    private static final double MAX_DRIVE_POWER = 0.5;
    private static final double kDrive          = 0.025;

    // ---------------- RPM Values ----------------
    private static final double FIXED_RPM  = 3800;
    private static final double TPR        = 28.0;

    private static double rpmToTPS(double rpm){ return rpm * TPR / 60.0; }
    private double launcherTargetTPS = rpmToTPS(FIXED_RPM);

    // ---------------- Auto State Machine ----------------
    private enum AutoState { AUTOSHOOT, FORWARD_30, DONE }
    private AutoState state = AutoState.AUTOSHOOT;
    private AutoState lastState = null;

    private final ElapsedTime autoStateTimer = new ElapsedTime();
    private double lastX = 0;
    private long wrongWayCountX = 0;

    @Override
    public void runOpMode() {

        // ---------------- MOTOR MAP ----------------
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

        initLauncher(leftLauncher,  true);
        initLauncher(rightLauncher, false);

        // Pinpoint
        initPinpoint();

        telemetry.addLine("READY: Forward + Shoot");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        autoStateTimer.reset();

        // ---------------- MAIN AUTO LOOP ----------------
        while (opModeIsActive() && state != AutoState.DONE) {

            if (pinpoint != null) pinpoint.update();

            if (state != lastState) {
                lastState = state;
                autoStateTimer.reset();
            }

            switch (state) {

                // ----------------------------------------------------
                // 1) Spin up launcher → feed note → stop
                // ----------------------------------------------------
                case AUTOSHOOT:
                    if (runAutoShootSimple()) {
                        stopDrive();
                        stopLaunch();
                        state = AutoState.FORWARD_30;
                    }
                    break;

                // ----------------------------------------------------
                // 2) Drive forward 30 inches (X increasing)
                // ----------------------------------------------------
                case FORWARD_30:
                    if (moveToX(30.0) || autoStateTimer.seconds() > 4.0) {
                        stopDrive();
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    stopDrive();
                    stopLaunch();
                    break;
            }

            telemetry.addData("State", state);
            if (pinpoint != null) {
                telemetry.addData("X (in)", "%.2f", pinpoint.getPosX(DistanceUnit.INCH));
            }
            telemetry.update();
        }

        stopDrive();
        stopLaunch();
    }

    // =========================================================================
    // SIMPLE AUTOSHOOT (NO TAGS)
    // =========================================================================

    private boolean runAutoShootSimple() {

        double elapsed = autoStateTimer.seconds();

        // 0–2.0 sec: spin up shooter
        if (elapsed < 2.0) {
            startLaunch();  // 3800 RPM
            return false;
        }

        // 2.0–3.2 sec: run intake to feed note
        if (elapsed < 3.2) {
            if (intake != null) intake.setPower(1.0);
            return false;
        }

        // Stop
        if (intake != null) intake.setPower(0);
        stopLaunch();
        return true;
    }

    private void startLaunch() {
        launcherTargetTPS = rpmToTPS(FIXED_RPM);
        if (leftLauncher != null)  leftLauncher.setVelocity(launcherTargetTPS);
        if (rightLauncher != null) rightLauncher.setVelocity(launcherTargetTPS);
    }

    private void stopLaunch() {
        if (leftLauncher != null)  leftLauncher.setPower(0);
        if (rightLauncher != null) rightLauncher.setPower(0);
    }

    // =========================================================================
    // MOVEMENT HELPERS
    // =========================================================================

    private double getX() {
        if (pinpoint == null) return 0;
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    private boolean moveToX(double targetXInches) {

        if (pinpoint == null) {
            // Fallback if no odometry
            drive(0.3, 0, 0);
            return autoStateTimer.seconds() > 2.0;
        }

        double currentX = getX();
        double error = targetXInches - currentX;
        double power = kDrive * error;

        power = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, power));

        // If within 1 inch, stop
        if (Math.abs(error) < 1.0) {
            drive(0, 0, 0);
            return true;
        }

        // Drive forward only
        drive(power, 0, 0);
        return false;
    }

    private void drive(double fwd, double strafe, double turn) {
        double fl = fwd + strafe + turn;
        double fr = fwd - strafe - turn;
        double bl = fwd - strafe + turn;
        double br = fwd + strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void stopDrive() {
        drive(0, 0, 0);
    }

    // =========================================================================
    // INIT HELPERS
    // =========================================================================

    private void initLauncher(DcMotorEx m, boolean reverse) {
        if (m == null) return;
        m.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        m.setZeroPowerBehavior(BRAKE);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setVelocityPIDFCoefficients(25, 0, 5, 18.0);
    }

    private void initPinpoint() {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(0, 0, DistanceUnit.MM);
            pinpoint.resetPosAndIMU();

            for (int i = 0; i < 25; i++) {
                pinpoint.update();
                sleep(20);
            }
            pinpoint.resetPosAndIMU();

        } catch (Exception e) {
            telemetry.addLine("Pinpoint NOT found; running without odometry.");
            telemetry.update();
            pinpoint = null;
        }
    }

    private DcMotor getMotor(String name) {
        try {
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            if (m != null) m.setZeroPowerBehavior(BRAKE);
            return m;
        } catch (Exception e) {
            return null;
        }
    }

    private DcMotorEx getMotorEx(String name) {
        try {
            DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
            if (m != null) m.setZeroPowerBehavior(BRAKE);
            return m;
        } catch (Exception e) {
            return null;
        }
    }
}

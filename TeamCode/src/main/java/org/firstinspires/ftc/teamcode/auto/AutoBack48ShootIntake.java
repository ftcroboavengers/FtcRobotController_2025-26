package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto: Back 48in, Shoot, Intake 5s", group = "RoboAvengers")
public class AutoBack48ShootIntake extends LinearOpMode {

    // -------- Adjust these for YOUR robot --------
    // Drivetrain geometry
    private static final double WHEEL_DIAMETER_IN = 4.0;    // goBILDA 96mm ≈ 3.78", use your wheel
    private static final double GEAR_REDUCTION   = 1.0;     // external reduction on drive (wheel rpm / motor rpm)
    private static final int    TICKS_PER_REV    = 537;     // 5202-0002/0003 Yellow Jacket (check yours!)
    private static final double INCHES_PER_TICK  = (Math.PI * WHEEL_DIAMETER_IN * GEAR_REDUCTION) / TICKS_PER_REV;

    // Motion
    private static final double DRIVE_POWER      = 0.4;     // comfortable power for straight drive
    private static final double SPINUP_SECONDS   = 1.0;     // time to let shooters reach velocity
    private static final double INTAKE_SECONDS   = 5.0;     // requested run time
    private static final double TIMEOUT_PER_FT   = 2.5;     // seconds/foot to prevent stalls (tune)

    // Shooter targets (ticks/second). Adjust if needed.
    private static final double LAUNCHER_TARGET  = 1200;    // your "close" preset

    // Hardware
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotorEx leftLauncher, rightLauncher;
    private DcMotor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        // ---- Map motors (use your names; fall back to alternates if needed) ----
        leftFront  = firstMotor("front_left_drive",  "frontLeftMotor");
        rightFront = firstMotor("front_right_drive", "frontRightMotor");
        leftBack   = firstMotor("back_left_drive",   "backLeftMotor");
        rightBack  = firstMotor("back_right_drive",  "backRightMotor");

        intake       = getMotor("intake");
        leftLauncher = getMotorEx("left_launcher");
        rightLauncher= getMotorEx("right_launcher");

        // ---- Directions & braking (matches your TeleOp) ----
        if (leftFront  != null) leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        if (leftBack   != null) leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        if (rightFront != null) rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        if (rightBack  != null) rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        setBrake(leftFront, rightFront, leftBack, rightBack);

        if (intake != null) intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Auto ready: Back 48in, Shoot, Intake 5s");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // ---- 1) Drive BACKWARDS ~48 inches (negative inches = backward) ----
        driveStraightInches(-48.0, DRIVE_POWER, TIMEOUT_PER_FT * (48.0 / 12.0));

        // ---- 2) Spin up launchers ----
        if (leftLauncher != null)  leftLauncher.setVelocity(LAUNCHER_TARGET);
        if (rightLauncher != null) rightLauncher.setVelocity(LAUNCHER_TARGET);
        sleep((long)(SPINUP_SECONDS * 1000));

        // ---- 3) Run intake for 5 seconds ----
        if (intake != null) intake.setPower(1.0);
        sleep((long)(INTAKE_SECONDS * 1000));

        // ---- 4) Stop all mechanisms ----
        if (intake != null) intake.setPower(0.0);
        if (leftLauncher != null)  leftLauncher.setPower(0.0);
        if (rightLauncher != null) rightLauncher.setPower(0.0);
        setDrivePower(0, 0, 0, 0);

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }

    // ----------------- Drive helpers -----------------
    private void driveStraightInches(double inches, double power, double timeoutSec) {
        if (!drivetrainReady()) return;

        int deltaTicks = (int) Math.round(inches / INCHES_PER_TICK);

        // Reset encoders
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set targets
        leftFront.setTargetPosition(deltaTicks);
        rightFront.setTargetPosition(deltaTicks);
        leftBack.setTargetPosition(deltaTicks);
        rightBack.setTargetPosition(deltaTicks);

        // RUN_TO_POSITION
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power (absolute value for RUN_TO_POSITION)
        double p = Math.abs(power);
        setDrivePower(p, p, p, p);

        // Wait until done or timeout
        long start = System.currentTimeMillis();
        while (opModeIsActive() && areMotorsBusy()
                && (System.currentTimeMillis() - start) < timeoutSec * 1000) {
            telemetry.addData("Target", deltaTicks);
            telemetry.addData("FL/FR", "%d / %d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.addData("BL/BR", "%d / %d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // Stop & switch back to RUN_USING_ENCODER
        setDrivePower(0, 0, 0, 0);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100); // small settle
    }

    private boolean drivetrainReady() {
        return leftFront != null && rightFront != null && leftBack != null && rightBack != null;
    }

    private boolean areMotorsBusy() {
        return leftFront.isBusy() || rightFront.isBusy() || leftBack.isBusy() || rightBack.isBusy();
    }

    private void setRunMode(DcMotor.RunMode mode) {
        if (leftFront  != null) leftFront.setMode(mode);
        if (rightFront != null) rightFront.setMode(mode);
        if (leftBack   != null) leftBack.setMode(mode);
        if (rightBack  != null) rightBack.setMode(mode);
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        if (leftFront  != null) leftFront.setPower(fl);
        if (rightFront != null) rightFront.setPower(fr);
        if (leftBack   != null) leftBack.setPower(bl);
        if (rightBack  != null) rightBack.setPower(br);
    }

    private void setBrake(DcMotor... motors) {
        for (DcMotor m : motors) if (m != null) m.setZeroPowerBehavior(BRAKE);
    }

    // ----------------- Safe getters -----------------
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

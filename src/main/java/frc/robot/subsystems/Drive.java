/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Pose2dWithCurvature;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.physics.DCMotorTransmission;
import frc.lib.physics.DifferentialDrive;
import frc.lib.trajectory.DistanceView;
import frc.lib.trajectory.PurePursuitController;
import frc.lib.trajectory.Trajectory;
import frc.lib.trajectory.TrajectoryIterator;
import frc.lib.trajectory.TrajectorySamplePoint;
import frc.lib.trajectory.TrajectoryUtil;
import frc.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import frc.lib.trajectory.timing.TimedState;
import frc.lib.trajectory.timing.TimingConstraint;
import frc.lib.trajectory.timing.TimingUtil;
import frc.lib.util.CheesyDriveHelper;
import frc.lib.util.DriveSignal;
import frc.lib.util.Units;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DrivetrainJoystick;

public class Drive extends Subsystem {
  private static final double DRIVE_ENCODER_PPR = 4096.;

  /**
   * One of the Drive train motors is a VictorSPX
   * 
   * private TalonSRX leftMaster, rightMaster, leftFollower, rightFollower;
   * 
   */

  private TalonSRX leftMaster, rightMaster, rightFollower;
  private VictorSPX leftFollower;

  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
  private PigeonIMU pigeon;
  public Rotation2d gyro_heading;
  private Rotation2d gyroOffset;
  public TimedState<Pose2dWithCurvature> path_setpoint;
  private boolean overrideTrajectory = false;

  // Drive Planner
  final DifferentialDrive mModel;
  private static final double kMaxDx = 2.0;
  private static final double kMaxDy = 0.25;
  private static final double kMaxDTheta = Math.toRadians(5.0);
  public Pose2d error;

  public enum FollowerType {
    FEEDFORWARD_ONLY, PURE_PURSUIT, PID, NONLINEAR_FEEDBACK
  }

  FollowerType mFollowerType = FollowerType.NONLINEAR_FEEDBACK;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING // velocity PID control
  }

  private DriveControlState driveControlState;

  /*
   * kF actually is used as arbitrary feedfoward and is calculated constantly when
   * following paths to monitor and control acceleration
   *
   * We actually tuned feed forward but not it should be 0 and tuned in the
   * DifferentialDrive class
   */
  // PID Right
  public static final double RIGHT_kF = 0; // 1023. / 3600
  public static final double RIGHT_kF_ = 0; // 1023. / 3600

  public static final double RIGHT_kP = 0.005, RIGHT_kI = 0.001, RIGHT_kD = 10;

  // PID Left
  public static final double LEFT_kF = 0; // 1023. / 3700
  public static final double LEFT_kP = 0.005, LEFT_kI = 0.001, LEFT_kD = 10;

  public Drive() {
    leftMaster = new TalonSRX(Constants.DRIVETRAIN_LEFT_MASTER_MOTOR_ID);
    leftMaster.configFactoryDefault();
    leftMaster.setSensorPhase(true);
    leftMaster.selectProfileSlot(0, 0);
    leftMaster.config_kP(0, LEFT_kP);
    leftMaster.config_kI(0, LEFT_kI);
    leftMaster.config_kD(0, LEFT_kD);
    leftMaster.config_kF(0, LEFT_kF);
    leftMaster.configNeutralDeadband(0.04, 0);

    // leftFollower = new TalonSRX(Constants.DRIVETRAIN_LEFT_FOLLOWER_MOTOR_ID);

    leftFollower = new VictorSPX(Constants.DRIVETRAIN_LEFT_FOLLOWER_MOTOR_ID);
    leftFollower.setInverted(false);
    leftFollower.configFactoryDefault();
    leftFollower.follow(leftMaster);
    leftFollower.configNeutralDeadband(0.04, 0);

    rightMaster = new TalonSRX(Constants.DRIVETRAIN_RIGHT_MASTER_MOTOR_ID);
    rightMaster.configFactoryDefault();
    rightMaster.setInverted(true);
    rightMaster.setSensorPhase(true);
    rightMaster.selectProfileSlot(0, 0);
    rightMaster.config_kP(0, RIGHT_kP);
    rightMaster.config_kI(0, RIGHT_kI);
    rightMaster.config_kD(0, RIGHT_kD);
    rightMaster.config_kF(0, RIGHT_kF);
    rightMaster.configNeutralDeadband(0.04, 0);

    rightFollower = new TalonSRX(Constants.DRIVETRAIN_RIGHT_FOLLOWER_MOTOR_ID);
    rightFollower.configFactoryDefault();
    rightFollower.setInverted(true);
    rightFollower.follow(rightMaster);
    rightFollower.configNeutralDeadband(0.04, 0);

    pigeon = new PigeonIMU(rightFollower); // Motor pigeon is attachd to

    final DCMotorTransmission transmission = new DCMotorTransmission(1.0 / Constants.kDriveKv,
        Units.inches_to_meters(Constants.kDriveWheelRadiusInches)
            * Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Constants.kRobotLinearInertia
            / (2.0 * Constants.kDriveKa),
        Constants.kDriveVIntercept);
    mModel = new DifferentialDrive(Constants.kRobotLinearInertia, Constants.kRobotAngularInertia,
        Constants.kRobotAngularDrag, Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
        Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor), transmission,
        transmission);
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

    error = Pose2d.identity();
    gyro_heading = Rotation2d.identity();
    gyroOffset = Rotation2d.identity();
    path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    if (Constants.DRIVETRAIN_DO_STUFF) {
      setDefaultCommand(new DrivetrainJoystick());
    }
  }

  public void drive(ControlMode controlMode, double left, double right) {
    leftMaster.set(controlMode, left);
    rightMaster.set(controlMode, right);
  }

  public void drive(ControlMode controlMode, DriveSignal driveSignal) {
    drive(controlMode, driveSignal.getLeft(), driveSignal.getRight());
  }

  public void driveVelocity(DriveSignal driveSignal, DriveSignal arbitraryFeedForward, DriveSignal acceleration) {
    leftMaster.set(ControlMode.Velocity, driveSignal.getLeft(), DemandType.ArbitraryFeedForward,
        arbitraryFeedForward.getLeft() + LEFT_kD * acceleration.getLeft() / 1023.0);
    rightMaster.set(ControlMode.Velocity, driveSignal.getRight(), DemandType.ArbitraryFeedForward,
        arbitraryFeedForward.getRight() + RIGHT_kD * acceleration.getRight() / 1023.0);
  }

  public void followPath() {
    if (driveControlState == DriveControlState.PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();

      Output output = update(now, Robot.robotState.getFieldToVehicle(now));

      // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
      // demand.right_feedforward_voltage / 12.0);

      error = error();
      path_setpoint = setpoint();

      if (!overrideTrajectory) {
        driveVelocity(
            new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
                radiansPerSecondToTicksPer100ms(output.right_velocity)),
            new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0),
            new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0,
                radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0));
      } else { // BRAAAAAKEEE
        driveVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE, DriveSignal.BRAKE);
      }
    } else {
      DriverStation.reportError("Drive is not in path following state", false);
    }
  }

  public void cheesyDrive(double throttle, double turn, boolean isQuickTurn) {
    drive(ControlMode.PercentOutput, cheesyDriveHelper.cheesyDrive(throttle, turn, isQuickTurn));
  }

  public synchronized void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0, 0, 0);
    rightMaster.setSelectedSensorPosition(0, 0, 0);
  }

  public void zeroSensors() {
    setHeading(Rotation2d.identity());
    resetEncoders();
    // mAutoShift = true;
  }

  public synchronized Rotation2d getHeading() {
    return Rotation2d.fromDegrees(pigeon.getFusedHeading()).rotateBy(gyroOffset);
  }

  public synchronized void setHeading(Rotation2d heading) {
    System.out.println("SET HEADING: " + heading.getDegrees());

    gyroOffset = heading.rotateBy(Rotation2d.fromDegrees(pigeon.getFusedHeading()).inverse());
    System.out.println("Gyro offset: " + gyroOffset.getDegrees());
  }

  private static double rotationsToInches(double rotations) {
    return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }

  private static double inchesToRotations(double inches) {
    return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
  }

  private static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
  }

  private static double radiansPerSecondToTicksPer100ms(double rad_s) {
    return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
  }

  public double getLeftEncoderRotations() {
    return leftMaster.getSelectedSensorPosition(0) / DRIVE_ENCODER_PPR;
  }

  public double getRightEncoderRotations() {
    return rightMaster.getSelectedSensorPosition(0) / DRIVE_ENCODER_PPR;
  }

  public double getLeftEncoderDistance() {
    return rotationsToInches(getLeftEncoderRotations());
  }

  public double getRightEncoderDistance() {
    return rotationsToInches(getRightEncoderRotations());
  }

  public double getLeftVelocityNativeUnits() {
    return leftMaster.getSelectedSensorVelocity(); // sensor units per 100ms
  }

  public double getRightVelocityNativeUnits() {
    return rightMaster.getSelectedSensorVelocity(); // sensor units per 100ms
  }

  public double getLeftLinearVelocity() {
    return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getRightLinearVelocity() {
    return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
  }

  public double getLinearVelocity() {
    return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
  }

  public double getAngularVelocity() {
    return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
  }

  public void overrideTrajectory(boolean value) {
    overrideTrajectory = value;
  }

  public double getLeftDriveLeadDistance() {
    return this.leftMaster.getSelectedSensorPosition();
  }

  public double getRightDriveLeadDistance() {
    return this.rightMaster.getSelectedSensorPosition();
  }

  public double getLeftDriveLeadVelocity() {
    return this.leftMaster.getSelectedSensorVelocity();
  }

  public double getRightDriveLeadVelocity() {
    return this.rightMaster.getSelectedSensorVelocity();
  }

  public TalonSRX createTalon(int id, boolean left) {
    TalonSRX talon = new TalonSRX(id);
    talon.set(ControlMode.PercentOutput, 0.0);
    talon.setInverted(left);
    return talon;
  }

  public TalonSRX createTalon(int id, boolean left, TalonSRX master) {
    TalonSRX talon = new TalonSRX(id);
    talon.set(ControlMode.Follower, 0.0);
    talon.setInverted(left);
    talon.follow(master);
    return talon;
  }

  // motion planning
  TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
  boolean mIsReversed = false;
  double mLastTime = Double.POSITIVE_INFINITY;
  public TimedState<Pose2dWithCurvature> mSetpoint = new TimedState<>(Pose2dWithCurvature.identity());
  Pose2d mError = Pose2d.identity();
  Output mOutput = new Output();

  DifferentialDrive.ChassisState prev_velocity_ = new DifferentialDrive.ChassisState();
  double mDt = 0.0;

  public void setTrajectory(final TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    mCurrentTrajectory = trajectory;
    mSetpoint = trajectory.getState();
    for (int i = 0; i < trajectory.trajectory().length(); ++i) {
      if (trajectory.trajectory().getState(i).velocity() > Util.kEpsilon) {
        mIsReversed = false;
        break;
      } else if (trajectory.trajectory().getState(i).velocity() < -Util.kEpsilon) {
        mIsReversed = true;
        break;
      }
    }
  }

  public void reset() {
    mError = Pose2d.identity();
    mOutput = new Output();
    mLastTime = Double.POSITIVE_INFINITY;
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints, double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    return generateTrajectory(reversed, waypoints, constraints, 0.0, 0.0, max_vel, max_accel, max_voltage);
  }

  public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed, final List<Pose2d> waypoints,
      final List<TimingConstraint<Pose2dWithCurvature>> constraints, double start_vel, double end_vel, double max_vel, // inches/s
      double max_accel, // inches/s^2
      double max_voltage) {
    List<Pose2d> waypoints_maybe_flipped = waypoints;
    final Pose2d flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
    // TODO re-architect the spline generator to support reverse.
    if (reversed) {
      waypoints_maybe_flipped = new ArrayList<>(waypoints.size());
      for (int i = 0; i < waypoints.size(); ++i) {
        waypoints_maybe_flipped.add(waypoints.get(i).transformBy(flip));
      }
    }

    // Create a trajectory from splines.
    Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints_maybe_flipped,
        kMaxDx, kMaxDy, kMaxDTheta);

    if (reversed) {
      List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
      for (int i = 0; i < trajectory.length(); ++i) {
        flipped.add(new Pose2dWithCurvature(trajectory.getState(i).getPose().transformBy(flip),
            -trajectory.getState(i).getCurvature(), trajectory.getState(i).getDCurvatureDs()));
      }
      trajectory = new Trajectory<>(flipped);
    }
    // Create the constraint that the robot must be able to traverse the trajectory
    // without ever applying more
    // than the specified voltage.
    final DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new DifferentialDriveDynamicsConstraint<>(
        mModel, max_voltage);
    List<TimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
    all_constraints.add(drive_constraints);
    if (constraints != null) {
      all_constraints.addAll(constraints);
    }
    // Generate the timed trajectory.
    Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(reversed,
        new DistanceView<>(trajectory), kMaxDx, all_constraints, start_vel, end_vel, max_vel, max_accel);
    return timed_trajectory;
  }

  // @Override
  // public String toCSV() {
  // DecimalFormat fmt = new DecimalFormat("#0.000");
  // return fmt.format(mOutput.left_velocity) + "," +
  // fmt.format(mOutput.right_velocity) + ","
  // + fmt.format(mOutput.left_feedforward_voltage) + "," +
  // fmt.format(mOutput.right_feedforward_voltage) + ","
  // + mSetpoint.toCSV();
  // }

  public static class Output {
    public Output() {
    }

    public Output(double left_velocity, double right_velocity, double left_accel, double right_accel,
        double left_feedforward_voltage, double right_feedforward_voltage) {
      this.left_velocity = left_velocity;
      this.right_velocity = right_velocity;
      this.left_accel = left_accel;
      this.right_accel = right_accel;
      this.left_feedforward_voltage = left_feedforward_voltage;
      this.right_feedforward_voltage = right_feedforward_voltage;
    }

    public double left_velocity; // rad/s
    public double right_velocity; // rad/s

    public double left_accel; // rad/s^2
    public double right_accel; // rad/s^2

    public double left_feedforward_voltage;
    public double right_feedforward_voltage;

    public void flip() {
      double tmp_left_velocity = left_velocity;
      left_velocity = -right_velocity;
      right_velocity = -tmp_left_velocity;

      double tmp_left_accel = left_accel;
      left_accel = -right_accel;
      right_accel = -tmp_left_accel;

      double tmp_left_feedforward = left_feedforward_voltage;
      left_feedforward_voltage = -right_feedforward_voltage;
      right_feedforward_voltage = -tmp_left_feedforward;
    }
  }

  protected Output updatePID(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
    DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
    // Feedback on longitudinal error (distance).
    final double kPathKX = 5.0;
    final double kPathKY = 1.0;
    final double kPathKTheta = 5.0;
    adjusted_velocity.linear = dynamics.chassis_velocity.linear
        + kPathKX * Units.inches_to_meters(mError.getTranslation().x());
    adjusted_velocity.angular = dynamics.chassis_velocity.angular
        + dynamics.chassis_velocity.linear * kPathKY * Units.inches_to_meters(mError.getTranslation().y())
        + kPathKTheta * mError.getRotation().getRadians();

    double curvature = adjusted_velocity.angular / adjusted_velocity.linear;
    if (Double.isInfinite(curvature)) {
      adjusted_velocity.linear = 0.0;
      adjusted_velocity.angular = dynamics.chassis_velocity.angular;
    }

    // Compute adjusted left and right wheel velocities.
    final DifferentialDrive.WheelState wheel_velocities = mModel.solveInverseKinematics(adjusted_velocity);
    final double left_voltage = dynamics.voltage.left
        + (wheel_velocities.left - dynamics.wheel_velocity.left) / mModel.left_transmission().speed_per_volt();
    final double right_voltage = dynamics.voltage.right
        + (wheel_velocities.right - dynamics.wheel_velocity.right) / mModel.right_transmission().speed_per_volt();

    return new Output(wheel_velocities.left, wheel_velocities.right, dynamics.wheel_acceleration.left,
        dynamics.wheel_acceleration.right, left_voltage, right_voltage);
  }

  protected Output updatePurePursuit(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
    double lookahead_time = Constants.kPathLookaheadTime;
    final double kLookaheadSearchDt = 0.01;
    TimedState<Pose2dWithCurvature> lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
    double actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
    while (actual_lookahead_distance < Constants.kPathMinLookaheadDistance
        && mCurrentTrajectory.getRemainingProgress() > lookahead_time) {
      lookahead_time += kLookaheadSearchDt;
      lookahead_state = mCurrentTrajectory.preview(lookahead_time).state();
      actual_lookahead_distance = mSetpoint.state().distance(lookahead_state.state());
    }
    if (actual_lookahead_distance < Constants.kPathMinLookaheadDistance) {
      lookahead_state = new TimedState<>(new Pose2dWithCurvature((Pose2d) (lookahead_state.state().getPose()
          .transformBy(Pose2d.fromTranslation(new Translation2d(
              (mIsReversed ? -1.0 : 1.0) * (Constants.kPathMinLookaheadDistance - actual_lookahead_distance), 0.0)))),
          0.0), lookahead_state.t(), lookahead_state.velocity(), lookahead_state.acceleration());
    }

    DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState();
    // Feedback on longitudinal error (distance).
    adjusted_velocity.linear = dynamics.chassis_velocity.linear
        + Constants.kPathKX * Units.inches_to_meters(mError.getTranslation().x());

    // Use pure pursuit to peek ahead along the trajectory and generate a new
    // curvature.
    final PurePursuitController.Arc<Pose2dWithCurvature> arc = new PurePursuitController.Arc<>(current_state,
        lookahead_state.state());

    double curvature = 1.0 / Units.inches_to_meters(arc.radius);
    if (Double.isInfinite(curvature)) {
      adjusted_velocity.linear = 0.0;
      adjusted_velocity.angular = dynamics.chassis_velocity.angular;
    } else {
      adjusted_velocity.angular = curvature * dynamics.chassis_velocity.linear;
    }

    dynamics.chassis_velocity = adjusted_velocity;
    dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);
    return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
        dynamics.wheel_acceleration.right, dynamics.voltage.left, dynamics.voltage.right);
  }

  protected Output updateNonlinearFeedback(DifferentialDrive.DriveDynamics dynamics, Pose2d current_state) {
    // Implements eqn. 5.12 from
    // https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
    final double kBeta = 2.0; // >0.
    final double kZeta = 0.7; // Damping coefficient, [0, 1].

    // Compute gain parameter.
    final double k = 2.0 * kZeta * Math.sqrt(kBeta * dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear
        + dynamics.chassis_velocity.angular * dynamics.chassis_velocity.angular);

    // Compute error components.
    final double angle_error_rads = mError.getRotation().getRadians();
    final double sin_x_over_x = Util.epsilonEquals(angle_error_rads, 0.0, 1E-2) ? 1.0
        : mError.getRotation().sin() / angle_error_rads;
    final DifferentialDrive.ChassisState adjusted_velocity = new DifferentialDrive.ChassisState(
        dynamics.chassis_velocity.linear * mError.getRotation().cos()
            + k * Units.inches_to_meters(mError.getTranslation().x()),
        dynamics.chassis_velocity.angular + k * angle_error_rads + dynamics.chassis_velocity.linear * kBeta
            * sin_x_over_x * Units.inches_to_meters(mError.getTranslation().y()));

    // Compute adjusted left and right wheel velocities.
    dynamics.chassis_velocity = adjusted_velocity;
    dynamics.wheel_velocity = mModel.solveInverseKinematics(adjusted_velocity);

    dynamics.chassis_acceleration.linear = mDt == 0 ? 0.0
        : (dynamics.chassis_velocity.linear - prev_velocity_.linear) / mDt;
    dynamics.chassis_acceleration.angular = mDt == 0 ? 0.0
        : (dynamics.chassis_velocity.angular - prev_velocity_.angular) / mDt;

    prev_velocity_ = dynamics.chassis_velocity;

    DifferentialDrive.WheelState feedforward_voltages = mModel.solveInverseDynamics(dynamics.chassis_velocity,
        dynamics.chassis_acceleration).voltage;

    return new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right, dynamics.wheel_acceleration.left,
        dynamics.wheel_acceleration.right, feedforward_voltages.left, feedforward_voltages.right);
  }

  public Output update(double timestamp, Pose2d current_state) {
    if (mCurrentTrajectory == null)
      return new Output();

    if (mCurrentTrajectory.getProgress() == 0.0 && !Double.isFinite(mLastTime)) {
      mLastTime = timestamp;
    }

    mDt = timestamp - mLastTime;
    mLastTime = timestamp;
    TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample_point = mCurrentTrajectory.advance(mDt);
    mSetpoint = sample_point.state();

    if (!mCurrentTrajectory.isDone()) {
      // Generate feedforward voltages.
      final double velocity_m = Units.inches_to_meters(mSetpoint.velocity());
      final double curvature_m = Units.meters_to_inches(mSetpoint.state().getCurvature());
      final double dcurvature_ds_m = Units
          .meters_to_inches(Units.meters_to_inches(mSetpoint.state().getDCurvatureDs()));
      final double acceleration_m = Units.inches_to_meters(mSetpoint.acceleration());
      final DifferentialDrive.DriveDynamics dynamics = mModel.solveInverseDynamics(
          new DifferentialDrive.ChassisState(velocity_m, velocity_m * curvature_m), new DifferentialDrive.ChassisState(
              acceleration_m, acceleration_m * curvature_m + velocity_m * velocity_m * dcurvature_ds_m));
      mError = current_state.inverse().transformBy(mSetpoint.state().getPose());

      if (mFollowerType == FollowerType.FEEDFORWARD_ONLY) {
        mOutput = new Output(dynamics.wheel_velocity.left, dynamics.wheel_velocity.right,
            dynamics.wheel_acceleration.left, dynamics.wheel_acceleration.right, dynamics.voltage.left,
            dynamics.voltage.right);
      } else if (mFollowerType == FollowerType.PURE_PURSUIT) {
        mOutput = updatePurePursuit(dynamics, current_state);
      } else if (mFollowerType == FollowerType.PID) {
        mOutput = updatePID(dynamics, current_state);
      } else if (mFollowerType == FollowerType.NONLINEAR_FEEDBACK) {
        mOutput = updateNonlinearFeedback(dynamics, current_state);
      }
    } else {
      // TODO Possibly switch to a pose stabilizing controller?
      mOutput = new Output();
    }
    return mOutput;
  }

  public boolean isDone() {
    return mCurrentTrajectory != null && mCurrentTrajectory.isDone();
  }

  public boolean isDoneWithTrajectory() {
    if (driveControlState != DriveControlState.PATH_FOLLOWING) {
      return false;
    }
    return this.isDone() || overrideTrajectory;
  }

  public void setDriveControlState(DriveControlState driveControlState) {
    this.driveControlState = driveControlState;
  }

  public DriveControlState getDriveControlState() {
    return driveControlState;
  }

  public Pose2d error() {
    return mError;
  }

  public TimedState<Pose2dWithCurvature> setpoint() {
    return mSetpoint;
  }

  public String periodicOutput() {
    return "";
  }
}

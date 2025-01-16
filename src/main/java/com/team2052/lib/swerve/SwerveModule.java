package com.team2052.lib.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveModule {
   private final TalonFX driveMotor;
   private final TalonFXConfiguration driveMotorConfig;
//    private final InvertedValue driveMotorInvertedValue;

   private final SparkMax steerMotor;
   private final SparkMaxConfig steerMotorConfig;
   private final CANcoder canCoder;
   private String debugName;

   public SwerveModule(
        String debugName,
        SwerveModuleConstants moduleConstants,
        int canCoderChannel
   ) {

    this.debugName = debugName;

    /*
    * CANCoder Initialization
    */
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    // canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; // replaced below
    canCoderConfiguration.MagnetSensor.MagnetOffset = (moduleConstants.steerOffset / (2 * Math.PI));
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    canCoder = new CANcoder(canCoderChannel);
    
    checkStatusCodeError( // worked without returning an error
        "Failed to configure CANCoder",
        canCoder.getConfigurator().apply(
            canCoderConfiguration,
            0.25
        )
    );

    /*
     * Drive Motor Initialization
     */

    driveMotor = new TalonFX(moduleConstants.driveMotorID);
    driveMotorConfig = new TalonFXConfiguration();
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    // TODO: clockwise vs counterclockwise positive - which is inverted
    // driveMotor.setInverted(SwerveConstants.SwerveModule.DRIVE_INVERTED); // old version
    driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // which one?

    VoltageConfigs driveVoltageConfig = new VoltageConfigs();
    driveVoltageConfig.withPeakForwardVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);
    driveVoltageConfig.withPeakReverseVoltage(SwerveConstants.MAX_VOLTAGE_VOLTS);

    try{
        driveMotor.getConfigurator().apply(
            driveVoltageConfig, 
            SwerveConstants.CAN_TIMEOUT_SECONDS);
    } catch(Error e){
        System.out.println("Failed to enable drive motor voltage compensation" + e.getMessage());
    }

    CurrentLimitsConfigs driveCurrentLimitConfig = new CurrentLimitsConfigs();
    driveCurrentLimitConfig.withSupplyCurrentLimitEnable(true);
    driveCurrentLimitConfig.withStatorCurrentLimit(SwerveConstants.DRIVE_STALL_CURRENT_LIMIT_AMPS);
    driveCurrentLimitConfig.withSupplyCurrentLimit(SwerveConstants.DRIVE_FREE_CURRENT_LIMIT_AMPS);

    try{
        driveMotor.getConfigurator().apply(
            driveCurrentLimitConfig,
            SwerveConstants.CAN_TIMEOUT_SECONDS
        );
    } catch(Error e){
        System.out.println("Failed to set steer motor current limit" + e.getMessage());
    }

    /*
     * Steer Motor Initialization
    */

    steerMotor = new SparkMax(moduleConstants.steerMotorID, MotorType.kBrushless);
    steerMotorConfig = new SparkMaxConfig();

    // don't need to restore defaults anymore
    // checkError("Failed to restore drive motor factory defaults", steerMotor.restoreFactoryDefaults());

    // don't need
    // checkError(
    //     "Failed to set drive motor periodic status frame rate",
    //     steerMotorConfig.signals.primaryEncoderPositionPeriodMs(100) // TODO: default kStatus 2??
    //     // steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100),
    //     // steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20),
    //     // steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20)
    // );

    steerMotorConfig.idleMode(IdleMode.kBrake);
    steerMotorConfig.inverted(SwerveConstants.SwerveModule.DRIVE_INVERTED);
    steerMotorConfig.voltageCompensation(SwerveConstants.MAX_VOLTAGE_VOLTS);
    steerMotorConfig.smartCurrentLimit((int)(SwerveConstants.STEER_CURRENT_LIMIT_AMPS));

    // Drive Motor encoder initialization
    RelativeEncoder steerEncoder = steerMotor.getEncoder();
    steerMotorConfig.encoder
            .positionConversionFactor(SwerveConstants.SwerveModule.steerPositionConversionFactor)
            .velocityConversionFactor(SwerveConstants.SwerveModule.steerPositionConversionFactor / 60.0);
    steerEncoder.setPosition(canCoder.getAbsolutePosition().getValueAsDouble() * (2 * Math.PI));
   
    steerMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    steerMotorConfig.closedLoop
        .pid(
            SwerveConstants.SwerveModule.STEER_MOTOR_P, 
            SwerveConstants.SwerveModule.STEER_MOTOR_I, 
            SwerveConstants.SwerveModule.STEER_MOTOR_D)
        .positionWrappingMinInput(-Math.PI)
        .positionWrappingMaxInput(Math.PI)
        .positionWrappingEnabled(true);

    try{
        steerMotor.configure(
            steerMotorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        System.out.println("!!!!!!!!!!!!!! CONFIGURE " + debugName);
    } catch(Exception e){
        System.out.println("FAILED TO CONFIGURE STEER MOTOR" + e.getMessage());
    }
}


    public SwerveModuleState getState() {
        // Both encoder values are automatically in units of meters per second and
        // radians because of the position and velocity conversion factors
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble(),
            new Rotation2d(
                (steerMotor.getEncoder().getPosition() % (2.0 * Math.PI))
            )
        );
    }

    public void setState(double velocityMetersPerSecond, Rotation2d steerAngle) { // in drive method
        SwerveModuleState desiredState = new SwerveModuleState(velocityMetersPerSecond, steerAngle);
        // Reduce radians to 0 to 2pi range and simplify to nearest angle
        desiredState.optimize(getState().angle);

        // Set the motor to our desired velocity as a percentage of our max velocity
        driveMotor.set(desiredState.speedMetersPerSecond / getMaxVelocityMetersPerSecond());

        steerMotor.getClosedLoopController().setReference(
            desiredState.angle.getRadians(), SparkMax.ControlType.kPosition
        );

        SmartDashboard.putNumber(debugName + ": Desired Rotation", steerAngle.getDegrees());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getRotorPosition().getValueAsDouble() * SwerveConstants.SwerveModule.drivePositionConversionFactor,
            new Rotation2d(
                steerMotor.getEncoder().getPosition()
            )
        );
    }

    public TalonFX getTalonFX(){
        return driveMotor;
    }

    public static double getMaxVelocityMetersPerSecond() {
        /*
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed (RPM)] / 60 * [Drive reduction] * [Wheel diameter (m)] * pi
         * This is a measure of how fast the robot should be able to drive in a straight line.
         */
        return SwerveConstants.SwerveModule.KRAKEN_ROUNDS_PER_MINUTE / 60 * SwerveConstants.SwerveModule.DRIVE_REDUCTION * 
            SwerveConstants.SwerveModule.WHEEL_DIAMETER_METERS * Math.PI;
    }

    public void debug() {
        SmartDashboard.putNumber(debugName + " Absolute CANcoder Degrees", canCoder.getAbsolutePosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber(debugName + " Current CANcoder Degrees", canCoder.getPosition().getValueAsDouble() * 360);
        SmartDashboard.putNumber(debugName + " Current Steer Encoder Degrees", Math.toDegrees(steerMotor.getEncoder().getPosition()));
    }

    @SuppressWarnings("unchecked")
  protected <E> void checkStatusCodeError(String message, StatusCode error) {
    if (error != StatusCode.OK) {
      DriverStation.reportError(
          message + " on [front right] module: " + error.toString(),
          true
      );
    }
  }

  protected <E> void checkREVError(String message, REVLibError error) {
    if (error != REVLibError.kOk) {
      DriverStation.reportError(
          message + " on [front right] module: " + error.toString(),
          true
      );
    }
  } 

    public static class SwerveModuleConstants {
		public final int driveMotorID;
		public final int steerMotorID;
		public final double steerOffset;

		public SwerveModuleConstants(int driveMotorID, int steerMotorID, double steerOffset) {
			this.driveMotorID = driveMotorID;
			this.steerMotorID = steerMotorID;
			this.steerOffset = steerOffset;
		}
	}
}  

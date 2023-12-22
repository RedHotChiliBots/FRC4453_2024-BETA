// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalIO;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LinearSlideConstants;
import frc.robot.Constants.ModuleConstants;

public class LinearSlide extends SubsystemBase {
	private boolean intializing = false;
	public final PowerDistribution pdh = new PowerDistribution(ModuleConstants.kPDH_CAN_ID, ModuleType.kRev);
	public final CANSparkMax motor = new CANSparkMax(DriveConstants.kLinearSlideCanId, MotorType.kBrushless);
	public final RelativeEncoder relEncoder = motor.getEncoder();
	private SparkMaxPIDController pidController = motor.getPIDController();
	private final DigitalInput digitalSwitch = new DigitalInput(DigitalIO.kDigitalSwitch);
	// private final Encoder thruEncoder = new Encoder(DigitalIO.kChannelA,
	// DigitalIO.kChannelB);
	private final ShuffleboardTab linearSlideTab = Shuffleboard.getTab("LinearSlide");
	private final GenericEntry sbPosition = linearSlideTab.addPersistent("Position", 0)
			.withWidget("Text View").withPosition(5, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbEncoderSpeed = linearSlideTab.addPersistent("Encoder Speed", 0)
			.withWidget("Text View").withPosition(5, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbMotorSpeed = linearSlideTab.addPersistent("Motor Speed", 0)
			.withWidget("Text View").withPosition(7, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbAMPS = linearSlideTab.addPersistent("AMPS", 0)
			.withWidget("Text View").withPosition(5, 2).withSize(2, 1).getEntry();
	private final GenericEntry sbRelativePOS = linearSlideTab.addPersistent("RelativePOS", 0)
			.withWidget("Text View").withPosition(5, 3).withSize(2, 1).getEntry();
	private final GenericEntry sbMTRPosition = linearSlideTab.addPersistent("MTRPosition", 0)
			.withWidget("Text View").withPosition(5, 4).withSize(2, 1).getEntry();
	private final GenericEntry sbSwitch = linearSlideTab.addPersistent("Switch", false)
			.withWidget("Boolean Box").withPosition(5, 5).withSize(2, 1).getEntry();
	private final GenericEntry sbJoystick = linearSlideTab.addPersistent("Joystick", 0)
			.withWidget("Text View").withPosition(3, 0).withSize(2, 1).getEntry();
	private final GenericEntry sbIsLSRev = linearSlideTab.addPersistent("Is SL Rev", false)
			.withWidget("Boolean Box").withPosition(7, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbIsLSFwd = linearSlideTab.addPersistent("Is SL Fwd", false)
			.withWidget("Boolean Box").withPosition(8, 0).withSize(1, 1).getEntry();
	private final GenericEntry sbLSRev = linearSlideTab.addPersistent("SL Rev", 0)
			.withWidget("Text View").withPosition(7, 1).withSize(2, 1).getEntry();
	private final GenericEntry sbLSFwd = linearSlideTab.addPersistent("SL Fwd", 0)
			.withWidget("Text View").withPosition(7, 2).withSize(2, 1).getEntry();

	public LinearSlide() {
		motor.setInverted(ModuleConstants.kLinearSlideInverted);
		motor.setIdleMode(ModuleConstants.kLinearSlideIdleMode);
		motor.setSmartCurrentLimit(ModuleConstants.kLinearSlideCurrentLimit);

		relEncoder.setPositionConversionFactor(LinearSlideConstants.posConversion);

		pidController.setP(LinearSlideConstants.kP);
		pidController.setI(LinearSlideConstants.kI);
		pidController.setD(LinearSlideConstants.kD);
		pidController.setIZone(LinearSlideConstants.kIz);
		pidController.setFF(LinearSlideConstants.kFF);
		pidController.setOutputRange(LinearSlideConstants.kMinOutput, LinearSlideConstants.kMaxOutput);

		pidController.setSmartMotionMaxVelocity(LinearSlideConstants.kMaxVel, LinearSlideConstants.kSmartMotionSlot);
		pidController.setSmartMotionMinOutputVelocity(LinearSlideConstants.kMinVel,
				LinearSlideConstants.kSmartMotionSlot);
		pidController.setSmartMotionMaxAccel(LinearSlideConstants.kMaxAcc, LinearSlideConstants.kSmartMotionSlot);
		pidController.setSmartMotionAllowedClosedLoopError(LinearSlideConstants.kAllowedErr,
				LinearSlideConstants.kSmartMotionSlot);
	}

	@Override
	public void periodic() {
		System.out.println("+++++ Starting LinearSlide Constructor +++++");

		sbPosition.setDouble(relEncoder.getPosition());
		sbEncoderSpeed.setDouble(relEncoder.getVelocity());
		sbMotorSpeed.setDouble(motor.get());
		sbAMPS.setDouble(pdh.getCurrent(5));
		sbSwitch.setBoolean(getSwitch());
		sbLSRev.setDouble(motor.getSoftLimit(SoftLimitDirection.kReverse));
		sbLSFwd.setDouble(motor.getSoftLimit(SoftLimitDirection.kForward));
		sbIsLSRev.setBoolean(motor.isSoftLimitEnabled(SoftLimitDirection.kReverse));
		sbIsLSFwd.setBoolean(motor.isSoftLimitEnabled(SoftLimitDirection.kForward));

		System.out.println("----- Ending LinearSlide Constructor -----");
	}

	public void joystickMovement(double xSpeed) {
		motor.set(xSpeed);
		sbJoystick.setDouble(xSpeed);
	}

	public void initializePositon() {
		motor.set(0.25);
		while (pdh.getCurrent(5) < 0.5) {
		}
		motor.set(0);
		relEncoder.setPosition(0);
	}

	public boolean getSwitch() {
		return !digitalSwitch.get();
	}

	public boolean getInitializing() {
		return intializing;
	}

	public void setInitializing(boolean intializing) {
		this.intializing = intializing;
	}

	public boolean initPos() {
		REVLibError status;
		boolean result = true;

		relEncoder.setPosition(0.0);

		status = motor.setSoftLimit(SoftLimitDirection.kReverse, 0.25f);
		if (status != REVLibError.kOk) {
			motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
			System.out.println("Linear Slide Reverse Soft Limit Enabled");
		} else {
			result = false;
			System.out.println("Linear Slide Reverse Soft Limit Failed: " + status.toString());
		}

		status = motor.setSoftLimit(SoftLimitDirection.kForward, 10.0f);
		if (status == REVLibError.kOk) {
			motor.enableSoftLimit(SoftLimitDirection.kForward, true);
			System.out.println("Linear Slide Forward Soft Limit Enabled");
		} else {
			result = false;
			System.out.println("Linear Slide Forward Soft Limit Failed: " + status.toString());
		}

		return result;
	}

	public void setPos(double pos) {
		pidController.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
	}

	public double getPos() {
		return relEncoder.getPosition();
	}
}

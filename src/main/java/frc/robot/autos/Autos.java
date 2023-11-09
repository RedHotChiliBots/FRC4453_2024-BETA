package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
	// Define Shuffleboard tab to hold competition info
	private final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

	// Define a chooser for autonomous commands
	private final SendableChooser<Command> chooser;

	// Define autonomous paths and commands
	private PathPlannerPath pathZigZag3m = null;
	private Command cmdZigZag3m = null;
	private Command cmdAutoZigZag3m = null;

	protected TrajectoryConfig config = new TrajectoryConfig(
			Constants.AutoConstants.kMaxSpeedMetersPerSecond,
			Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			.setKinematics(Constants.DriveConstants.kDriveKinematics);

	protected ProfiledPIDController thetaController = new ProfiledPIDController(
			Constants.AutoConstants.kPThetaController, 0, 0,
			Constants.AutoConstants.kThetaControllerConstraints);

	protected HolonomicDriveController holonomicController = new HolonomicDriveController(
			new PIDController(Constants.AutoConstants.kPXController, 0, 0),
			new PIDController(Constants.AutoConstants.kPYController, 0, 0),
			thetaController);

	// An example trajectory to follow. All units in meters.
	protected Trajectory zigzag3Trajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			config);

	public Autos(Chassis chassis) {
		System.out.println("+++++ Starting Autos Constructor +++++");

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		// Configure the AutoBuilder fpr Swerve
		AutoBuilder.configureHolonomic(
				chassis::getPose, // Robot pose supplier
				chassis::resetPose, // Method to reset odometry (will be called if your auto has a
														// starting pose)
				chassis::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				chassis::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE
																			// ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
																					// in your Constants class
						new PIDConstants(ModuleConstants.kDrivingP,
								0.0, 0.0), // Translation PID constants
						new PIDConstants(ModuleConstants.kDrivingP,
								0.0, 0.0), // Rotation PID constants
						DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						DriveConstants.kWheelRadius, // Drive base radius in meters. Distance
																					// from robot center to
																					// furthest module.
						new ReplanningConfig() // Default path replanning config. See the API
																		// for the options here
				),
				chassis // Reference to this subsystem to set requirements
		);

		// Register Named Commands
//		NamedCommands.registerCommand("ZigZag3m", cmdZigZag3m);

		String temp = AutoBuilder.isConfigured() ? "IS" : "IS NOT";
		System.out.println("AutoBuilder " + temp + " configured");
		temp = AutoBuilder.isPathfindingConfigured() ? "IS" : "IS NOT";
		System.out.println("AutoBuilder Pathfinding " + temp + " configured");

		// Initialize auton paths and autos
		String pathStr = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "pathplanner"
				+ File.separator + "paths";
		File paths = new File(pathStr);
		System.out.println("Using Paths from default base location: " + paths.getAbsolutePath());
		File[] pathList = paths.listFiles();
		for (File file : pathList) {
			System.out.println("\t" + file.getName());
		}

		String autoStr = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "pathplanner"
				+ File.separator + "autos";
		File autos = new File(autoStr);
		System.out.println("Using Autos from default base location: " + autos.getAbsolutePath());
		File[] autoList = autos.listFiles();
		for (File file : autoList) {
			System.out.println("\t" + file.getName());
		}

		pathZigZag3m = PathPlannerPath.fromPathFile("ZigZag3m");
		if (pathZigZag3m == null)
			System.out.println("Path is NULL");
		int pts = pathZigZag3m.numPoints();
		List<EventMarker> events = pathZigZag3m.getEventMarkers();
		System.out.println("Path ZigZag3m has " + pts + " points with " + events.size() + " events.");

		cmdZigZag3m = AutoBuilder.followPathWithEvents(pathZigZag3m);
		cmdAutoZigZag3m = new PathPlannerAuto("ZigZag3m");

		// Initialize auton chooser with auton commands
		chooser = AutoBuilder.buildAutoChooser();
//		chooser = new SendableChooser<>();

//		chooser.setDefaultOption("None", Commands.none());
//		chooser.addOption("ZigZag3", cmdZigZag3m);

		// Add Auton Command chooser to Shuffleboard
		compTab.add("Auton Command", chooser)
				.withWidget("ComboBox Chooser")
				.withPosition(0, 0)
				.withSize(4, 1);

		System.out.println("----- Ending Autos Constructor -----");
	}

	public SendableChooser<Command> getChooser() {
		return chooser;
	}

	// public static PathPlannerTrajectory scoreElement1 =
	// PathPlanner.loadPath("ScoreElement1",
	// new PathConstraints(4, 3));

	// public static PathPlannerTrajectory selectElement2 =
	// PathPlanner.loadPath("SelectElement2",
	// new PathConstraints(4, 3));
	// public static PathPlannerTrajectory scoreElement2 =
	// PathPlanner.loadPath("ScoreElement2",
	// new PathConstraints(4, 3));

	// SwerveControllerCommand swerveControllerCommand =
	// new SwerveControllerCommand(
	// exampleTrajectory,
	// s_Drive::getPose,
	// Constants.DriveConstants.kDriveKinematics,
	// holonomicController,
	// s_Drive::setModuleStates,
	// s_Drive);

	// addCommands(
	// new InstantCommand(() ->
	// s_Drive.resetOdometry(exampleTrajectory.getInitialPose())),
	// swerveControllerCommand
	// );
}
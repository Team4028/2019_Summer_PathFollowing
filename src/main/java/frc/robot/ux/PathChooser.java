package frc.robot.ux;

import java.io.File;
import java.io.FilenameFilter;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.entities.LogDataBE;
import frc.robot.interfaces.IBeakSquadDataPublisher;

/**
 * This class dynamically builds the Auto Choosers from the path files on the RoboRio
 */
public class PathChooser implements IBeakSquadDataPublisher {

    private SendableChooser _pathAction = new SendableChooser<>();

    private static final String PATHS_FOLDER = "/home/lvuser/deploy/paths/output";

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static PathChooser _instance = new PathChooser();

    public static PathChooser getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private PathChooser() {
        // get the folder on the RoboRio that contains the paths
        File directory = new File(PATHS_FOLDER);

        // get a list of filenames that match the filter
        List<File> list = Arrays.asList(directory.listFiles(new FilenameFilter(){
            @Override
            public boolean accept(File dir, String name) {
                return name.endsWith(".pf1.csv"); // or something else
            }}));

        // sort Ascending on filename
        list.sort(Comparator.comparing(File::getName));

        // build chooser with auton options
        String fileName;
        String fileDisplayName;
        _pathAction.setDefaultOption("Do Nothing", "DO_NOTHING");
        for (File autonFile : list) {
            fileName = autonFile.getName();
            fileDisplayName = fileName.replace("_", "");
			_pathAction.addOption(fileDisplayName, fileName);
		}
    }

    @Override
    public void updateDashboard() {
       SmartDashboard.putData("Paths", _pathAction);
       SmartDashboard.putString("PathChooser:PathAction", get_AutonPathName());

    }

    public String get_AutonPathName()
    {
        return _pathAction.getSelected().toString();
    }

    @Override
    public void updateLogData(LogDataBE logData, boolean isVerboseLoggingEnabled) {
        // do nothing
    }
}

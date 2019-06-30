package frc.robot.ux;

import java.io.File;
import java.util.Hashtable;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.entities.LogDataBE;
import frc.robot.interfaces.IBeakSquadDataPublisher;

/**
 * This class defines all of the auton choosers tha will be available on the
 * Dashboard
 */
public class PathChooser implements IBeakSquadDataPublisher {

    private SendableChooser<String> _pathAction = new SendableChooser<>();

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static PathChooser _instance = new PathChooser();

    public static PathChooser getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private PathChooser() {
        File directory = new File("/home/lvuser/deploy/paths/output");
        Hashtable<String, String> ht = new Hashtable<String, String>();
        String[] paths = directory.list();

        ht.put("Do Nothing", "Do_Nothing"); // Add Do_Nothing option into the hashtable

        for(int i = 0; i < paths.length; i++){
            String fileName = paths[i].split("[.]")[0];
            if(!ht.containsKey(fileName)) {
                ht.put(fileName, fileName.replace("_", " "));
            }
        }
        //ht.forEach((k,v) -> System.out.println("Key : " + k + ", Value : " + v));

        // Auton Mode
        _pathAction.setDefaultOption("Do Nothing", ht.get("Do_Nothing")); //Set Do_Nothing as the default
        ht.forEach((k,v) -> _pathAction.addOption(v, k)); //Put all options in Hashtable onto dashboard
    }

    public String getSelectedPath() {
        return _pathAction.getSelected().toString();
    }



    @Override
    public void updateDashboard() {
       SmartDashboard.putData("Paths", _pathAction);
       SmartDashboard.putString("PathChooser:PathAction", _pathAction.getSelected().toString());
    }

    @Override
    public void updateLogData(LogDataBE logData) {

    }
}

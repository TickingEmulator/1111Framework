package frc.robot.Miscellaneous;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Miscellaneous.Constants.OIConstants;

/*
 * Two warnings will appear due to driver and operator fields not in use at the beginning.
 * The warnings should disappear when button configurations are applied.
 */
public class OI {
    /**
     * Creates two Joystick objects for each player and adds subsystems
     */
    private Joystick driver;
    private Joystick operator;
    /**
     * Instantiating all objects and applying configurations
     */
    public OI(){
        driver = new Joystick(OIConstants.kDriverID);
        operator = new Joystick(OIConstants.kOperatorID);
        configureJoysticks();
    }
    /**
     * Maps Joysticks to button integers and commands
     */
    private void configureJoysticks(){}
}

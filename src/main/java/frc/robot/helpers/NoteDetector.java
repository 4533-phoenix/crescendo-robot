package frc.robot.helpers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The class for the note detector.
 */
public class NoteDetector {
    /**
     * The instance of the {@link NetworkTableInstance} class.
     */
    private static final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

    /**
     * The instance of the {@link NetworkTable} class.
     */
    private NetworkTable networkTable;

    /**
     * The constructor for the {@link NoteDetector} class.
     * 
     * @param name The name of the network table.
     */
    public NoteDetector(String name) {
        networkTable = networkTableInstance.getTable(name);
    }

    /**
     * Gets if the camera can see a note
     * 
     * @return If the camera can see a note.
     */
    public boolean canSeeNote() {
        return networkTable.getEntry("note").getBoolean(false);
    }

    /**
     * Gets the x coordinate of the note.
     * 
     * @return The x coordinate of the note.
     */
    public double getNoteX() {
        return networkTable.getEntry("x").getDouble(0.0);
    }

    /**
     * Gets the y coordinate of the note.
     * 
     * @return The y coordinate of the note.
     */
    public double getNoteY() {
        return networkTable.getEntry("y").getDouble(0.0);
    }

    /**
     * Gets the x and y coordinates of the note.

     * @return The x and y coordinates of the note.
     */
    public double[] getNoteCoordinates() {
        return new double[] { getNoteX(), getNoteY() };
    }

    // width and height
    /**
     * Gets the width of the note.
     * 
     * @return The width of the note.
     */
    public double getNoteWidth() {
        return networkTable.getEntry("width").getDouble(0.0);
    }

    /**
     * Gets the height of the note.
     * 
     * @return The height of the note.
     */
    public double getNoteHeight() {
        return networkTable.getEntry("height").getDouble(0.0);
    }

    /**
     * Gets the width and height of the note.
     * 
     * @return The width and height of the note.
     */
    public double[] getNoteDimensions() {
        return new double[] { getNoteWidth(), getNoteHeight() };
    }

    /**
     * Gets the timestamp of when the last camera
     * frame was processed.
     * 
     * @return The timestamp of when the last
     * camera frame was processed.
     */
    public double getTimestamp() {
        return networkTable.getEntry("timestamp").getDouble(0.0);
    }
}

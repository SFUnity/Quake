package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

// import frc.robot.Constants.DriveConstants;

public class LimelightSubsystem extends SubsystemBase {

  private static LimelightSubsystem instance = null;

  public ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

  //Declaring objects that are used for retrieving data from the limelight.

  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry tv;
  private static NetworkTableEntry ta;
  private static NetworkTableEntry camMode;
  private static NetworkTableEntry ledMode;

  private GenericEntry txEntry;

  private LimelightSubsystem ()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
    ty = table.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
    tv = table.getEntry("tv"); // Whether the limelight has any valid targets (0 or 1).
    ta = table.getEntry("ta"); // Target area (0% of image to 100% of image).
    ledMode = table.getEntry("ledMode"); // limelight's LED state (0-3).
    camMode = table.getEntry("camMode"); // limelight's operation mode (0-1).

    txEntry = limelightTab.add("tx", 0).getEntry();
    // limelightTab.add("ty", ty).getEntry();
    // limelightTab.add("tv", tv).getEntry();
    // limelightTab.add("ta", ta).getEntry();
  }

  @Override
  public void periodic() {
      super.periodic();
      txEntry.setDouble(table.getEntry("tx").getDouble(0));
  }

  /**
   * Horizontal offset from crosshair to target.
   * @return offset from -29.8 to 29.8 degrees.
   */
  public double getTargetOffsetX()
  {
    return tx.getDouble(0.0);
  }

  /**
   * Vertical offset from crosshair to target.
   * @return offset from -24.85 to 24.85 degrees.
   */
  public double getTargetOffsetY()
  {
    return ty.getDouble(0.0);
  }

  /**
   * Get whether or not a target is detected.
   * @return true if target is found and false if target is not found.
   */
  public boolean isTargetAvailable()
  {
    return tv.getNumber(0).intValue() == 1 ? true : false;
  }

  /**
   * Get area of detected target.                                                                                                                        
   * @return target area from 0% to 100%.
   */
  public double getTargetArea()
  {
    return ta.getDouble(0.0);
  }

  /**
   * Method for other classes to use this class' methods.
   * @return the limelight instance object.
   */
  public static LimelightSubsystem getInstance()
  {
    if (instance == null)
      instance = new LimelightSubsystem();
    
    return instance;
  }
}

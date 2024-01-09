package frc.robot.subsystems.drive;

import java.io.File;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import com.fasterxml.jackson.annotation.JsonValue;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.Getter;

@Getter
@JsonIgnoreProperties(ignoreUnknown = true)
public class PivotConfig {
    
    public static enum PivotId { FL, FR, BL, BR;
        @JsonValue public String getName() { return this.name(); }
    }

    private static Map<PivotId,PivotConfig> cfgMap;

    private PivotId name;
    private Translation2d position;
    private double minvoltage;
    private double maxvoltage;
    private int driveChannel;
    private int steerChannel;
    private int resolverChannel;
    private double offset;
    private double rpmMax;
    private boolean reverseDrive;
    private boolean reverseSteer;
    private boolean reverseAngle;

    static {
        try {
            String cfgPath = Filesystem.getDeployDirectory() + "/pivotcfg.json";
            cfgMap = new ObjectMapper().readValue(new File(cfgPath), new TypeReference<Map<PivotId,PivotConfig>>(){});
        } catch (Exception e) {
            cfgMap = null;
            e.printStackTrace();
        }
    }

    public static PivotConfig getConfig (PivotId pivotId) {
        return cfgMap.get(pivotId);
    }
}

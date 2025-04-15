package frc.robot.utility;

import java.util.function.Supplier;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class SparkMaxUtil {
    public static boolean configureSpark(String message, Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < 5; i++) {
            err = config.get();
            if (err == REVLibError.kOk) {
                return true;
            }
        }

        DriverStation.reportError(String.format(
            "[MergeError] - CANSparkMax failed to configure setting. MergeMessage: %s. Spark error code: %s \nSee stack trace below.", 
            message,
            err.toString()), 
            true);
            
        return false;
    }
}

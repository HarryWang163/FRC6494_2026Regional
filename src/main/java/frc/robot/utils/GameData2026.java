package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameData2026 {
    private static final double SHIFT_1_END_TIME = 105.0;
    private static final double SHIFT_2_END_TIME = 80.0;
    private static final double SHIFT_3_END_TIME = 55.0;
    private static final double SHIFT_4_END_TIME = 30.0;
    private static final double TRANSITION_END_TIME = 130.0;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("GameData2026");

    public void periodic() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();

        HubWindow window = calculateHubWindow(alliance, matchTime, gameData);

        table.getEntry("HubActive").setBoolean(window.hubActive);
        table.getEntry("HubStatus").setString(window.status);
        table.getEntry("SecondsUntilHubOpens").setDouble(window.secondsUntilHubOpens);
        table.getEntry("SecondsUntilHubChanges").setDouble(window.secondsUntilHubChanges);
        table.getEntry("Shift").setString(window.shift);
        table.getEntry("MatchTime").setDouble(matchTime);
        table.getEntry("GameData").setString(gameData);
        table.getEntry("Alliance").setString(alliance.map(Enum::name).orElse("Unknown"));
        table.getEntry("FirstInactiveAlliance").setString(getFirstInactiveAlliance(gameData));
    }

    private HubWindow calculateHubWindow(Optional<Alliance> alliance, double matchTime, String gameData) {
        if (alliance.isEmpty()) {
            return new HubWindow(false, "NO_ALLIANCE", "Disabled", Double.NaN, Double.NaN);
        }

        if (DriverStation.isAutonomousEnabled()) {
            return new HubWindow(true, "OPEN", "Auto", 0.0, Double.NaN);
        }

        if (!DriverStation.isTeleopEnabled()) {
            return new HubWindow(false, "DISABLED", "Disabled", Double.NaN, Double.NaN);
        }

        if (matchTime < 0.0) {
            return new HubWindow(true, "OPEN_TIME_UNKNOWN", "Unknown", 0.0, Double.NaN);
        }

        Boolean redInactiveFirst = getRedInactiveFirst(gameData);
        if (redInactiveFirst == null) {
            return new HubWindow(true, "WAITING_DATA", "WaitingData", 0.0, Double.NaN);
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > TRANSITION_END_TIME) {
            return new HubWindow(true, "OPEN", "Transition", 0.0, matchTime - TRANSITION_END_TIME);
        } else if (matchTime > SHIFT_1_END_TIME) {
            return fromShift("Shift1", shift1Active, matchTime, SHIFT_1_END_TIME);
        } else if (matchTime > SHIFT_2_END_TIME) {
            return fromShift("Shift2", !shift1Active, matchTime, SHIFT_2_END_TIME);
        } else if (matchTime > SHIFT_3_END_TIME) {
            return fromShift("Shift3", shift1Active, matchTime, SHIFT_3_END_TIME);
        } else if (matchTime > SHIFT_4_END_TIME) {
            return fromShift("Shift4", !shift1Active, matchTime, SHIFT_4_END_TIME);
        } else {
            return new HubWindow(true, "OPEN", "Endgame", 0.0, Math.max(matchTime, 0.0));
        }
    }

    private HubWindow fromShift(String shift, boolean hubActive, double matchTime, double nextBoundary) {
        double secondsUntilChange = Math.max(matchTime - nextBoundary, 0.0);
        double secondsUntilOpen = hubActive ? 0.0 : secondsUntilChange;
        String status = hubActive ? "OPEN" : "CLOSED";
        return new HubWindow(hubActive, status, shift, secondsUntilOpen, secondsUntilChange);
    }

    private Boolean getRedInactiveFirst(String gameData) {
        if (gameData.isEmpty()) {
            return null;
        }

        return switch (gameData.charAt(0)) {
            case 'R' -> true;
            case 'B' -> false;
            default -> null;
        };
    }

    private String getFirstInactiveAlliance(String gameData) {
        if (gameData.isEmpty()) {
            return "None";
        }

        return switch (gameData.charAt(0)) {
            case 'R' -> "Red";
            case 'B' -> "Blue";
            default -> "Invalid";
        };
    }

    private record HubWindow(
        boolean hubActive,
        String status,
        String shift,
        double secondsUntilHubOpens,
        double secondsUntilHubChanges
    ) {}
}

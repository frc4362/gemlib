package com.gemsrobotics.frc2022.subsystems;

import java.util.Objects;
import java.util.Optional;

import com.gemsrobotics.frc2022.Constants;
import com.gemsrobotics.lib.data.RollingAverage;
import com.gemsrobotics.lib.drivers.PicoColorSensor;
import com.gemsrobotics.lib.drivers.PicoColorSensor.RawColor;
import com.gemsrobotics.lib.structure.Subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class CargoObserver extends Subsystem {
    private static CargoObserver INSTANCE;

    public static CargoObserver getInstance() {
        if (Objects.isNull(INSTANCE)) {
            INSTANCE = new CargoObserver();
        }

        return INSTANCE;
    }

    private final PicoColorSensor m_colorSensor;
    private final RollingAverage<RawColor> m_rollingAverage;
    private boolean m_filterEnabled;

    private CargoObserver() {
        m_colorSensor = new PicoColorSensor();
        m_rollingAverage = new RollingAverage<RawColor>(5);
        m_rollingAverage.add(new RawColor(0, 0, 0, 0));
        m_filterEnabled = Constants.DO_CARGO_REJECT;
    }

    @Override
    protected void readPeriodicInputs(final double timestamp) {
        m_rollingAverage.add(m_colorSensor.getRawColor0());
    }

    @Override
    protected void onStart(final double timestamp) {
        m_rollingAverage.clear();
        m_rollingAverage.add(new RawColor(0, 0, 0, 0));
    }

    @Override
    protected void onUpdate(final double timestamp) {
        
    }

    @Override
    protected void onStop(final double timestamp) {
        m_rollingAverage.clear();
        m_rollingAverage.add(new RawColor(0, 0, 0, 0));
    }

    public Optional<Alliance> getObservedCargoColor() {
        if (m_rollingAverage.size() < 5 || getFilteredBrightness() < 1500) {
            return Optional.empty();
        } else {
            final var avg = m_rollingAverage.getAverage();
            return Optional.of(avg.red > avg.blue ? Alliance.Red : Alliance.Blue);
        }
    }

    public int getFilteredBrightness() {
        if (m_rollingAverage.size() == 0) {
            return 0;
        } else {
            return m_rollingAverage.getAverage().getBrightness();
        }
    }

    @Override
    public void setSafeState() {
        
    }

    public Alliance getCargoAlliance() {
        return getObservedCargoColor().orElse(Alliance.Invalid);
    }

    public boolean isBadCargo() {
        final var detectedAlliance = getCargoAlliance();
        return m_filterEnabled && (detectedAlliance != DriverStation.getAlliance() && detectedAlliance != Alliance.Invalid);
        // return false;
    }

    public RawColor getFilteredColor() {
        return m_rollingAverage.getAverage();
    }

    public void setFilterDisabled() {
        m_filterEnabled = false;
    }

    public void setFilterDefault() {
        m_filterEnabled = Constants.DO_CARGO_REJECT;
    }
}

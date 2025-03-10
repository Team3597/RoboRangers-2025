package lib.frc3597.looper;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants;

public class Looper extends BasicLooperRunnable {
    private final Notifier notifier;
    private final double period = Constants.LOOPER_DELTA_DURATION;

    public Looper() {
        this.notifier = new Notifier(this);
    }

    @Override
    public synchronized void register(ILoop system) {
        super.register(system);
    }

    public void start() {
        if (!mRunning) {
            super.start();
            notifier.startPeriodic(period);
        }
    }

    public void run() {
        super.run();
    }

    public void stop() {
        if (mRunning) {
            super.stop();
            notifier.stop();
        }
    }
}

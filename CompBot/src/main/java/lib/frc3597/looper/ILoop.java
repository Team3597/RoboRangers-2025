package lib.frc3597.looper;

public interface ILoop {
    void onStart(double timestamp);
    void onLoop(double timestamp);
    void onStop(double timestamp);
}

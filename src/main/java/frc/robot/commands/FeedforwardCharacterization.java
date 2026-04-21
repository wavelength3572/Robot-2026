package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

/** Utility for measuring feedforward constants (kS and kV) via a voltage ramp. */
public final class FeedforwardCharacterization {

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private FeedforwardCharacterization() {}

  /**
   * Build a feedforward characterization command for any velocity-controlled subsystem.
   *
   * @param subsystem Subsystem to require (prevents other commands from running it)
   * @param voltageApplier Applies the ramp voltage to the subsystem hardware
   * @param velocityReader Reads the subsystem's current velocity (in native units/sec)
   * @param subsystemName Label used in the printed results
   */
  public static Command run(
      SubsystemBase subsystem,
      Consumer<Double> voltageApplier,
      DoubleSupplier velocityReader,
      String subsystemName) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),
        Commands.run(() -> voltageApplier.accept(0.0), subsystem).withTimeout(FF_START_DELAY),
        Commands.runOnce(timer::restart),
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  voltageApplier.accept(voltage);
                  velocitySamples.add(velocityReader.getAsDouble());
                  voltageSamples.add(voltage);
                },
                subsystem)
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println(
                      "********** " + subsystemName + " FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}

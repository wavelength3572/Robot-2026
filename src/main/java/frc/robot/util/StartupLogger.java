package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

/** Buffers startup log messages so they can be printed together after initialization completes. */
public class StartupLogger {
  private static final List<String> messages = new ArrayList<>();
  private static boolean flushed = false;

  /** Queue a message to be printed later during flush. */
  public static void log(String message) {
    if (flushed) {
      // After flush, just print immediately
      System.out.println(message);
    } else {
      messages.add(message);
    }
  }

  /** Print all buffered messages and switch to immediate mode. */
  public static void flush() {
    for (String msg : messages) {
      System.out.println(msg);
    }
    messages.clear();
    flushed = true;
  }
}

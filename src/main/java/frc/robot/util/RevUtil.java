package frc.robot.util;

import com.revrobotics.REVLibError;

public class RevUtil {
  public static REVLibError checkRevError(REVLibError error) {
    if (error == REVLibError.kOk) return error;
    System.out.println(error);
    return error;
  }
}

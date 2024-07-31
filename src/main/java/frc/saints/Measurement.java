
package frc.saints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Measurement {
    public double timestamp;
    public Pose2d pose;
    public Matrix<N3, N1> stdDeviation;

    public Measurement(double timestamp, Pose2d pose, Matrix<N3, N1> stdDeviation) {
      this.timestamp = timestamp;
      this.pose = pose;
      this.stdDeviation = stdDeviation;
    }

    public Pose2d getPose2d(){
      return pose;
    }

    public double getTimestamp(){
      return timestamp;
    }

    public Matrix<N3, N1> getDeviation(){
      return stdDeviation;
    }
  }

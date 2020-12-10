#ifndef PID_h
#define PID_h

template <typename T>
class PID {
private:
  T sum = 0; // sum for I term
  T last_err = 0; // last error
  T last_target = 0; // and target
  T last_output = 0;

public:
  float kp, ki, kd;
  T max_ramp_rate;
  T out_max, out_min;
  
  T update (T target, T feedback) {
    T err = target - feedback;

    T output = 0;
    // Proportional
    output += err * kp;

    // Integral
    if (ki) {
      sum += err * ki;

      if (sum > out_max) sum = out_max;
      else if (sum < out_min) sum = out_min;

      output += sum;
    }

    // Derivative
    if (kd) {
      T deriv = (err - last_err);
      last_err = err;
      last_target = target;

      output += deriv * kd;
    }

    if (output > out_max) output = out_max;
    else if (output < out_min) output = out_min;

    if (output > last_output + max_ramp_rate) output = last_output + max_ramp_rate;
    else if (output < last_output - max_ramp_rate) output = last_output - max_ramp_rate;
    last_output = output;

    return output;
  }
};

#endif

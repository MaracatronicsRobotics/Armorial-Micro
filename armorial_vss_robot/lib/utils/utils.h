#ifndef ARMORIAL_SUASSUNA_UTILS_H
#define ARMORIAL_SUASSUNA_UTILS_H

class Utils {
public:
  static float fmap(float x, float in_min, float in_max, float out_min,
                    float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

#endif // ARMORIAL_SUASSUNA_UTILS_H

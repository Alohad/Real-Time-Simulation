#ifndef VECTOR3_H
#define VECTOR3_H
class Vector3 {

public:

  Vector3() {}

  Vector3(float _x, float _y, float _z) :
     x_(_x), y_(_y), z_(_z) {}

  float GetX() const { return x_; }

  float GetY() const { return y_; }

  float GetZ() const { return z_; }

  void SetX(const float& x) { x_ = x; }

  void SetY(const float& y) { y_ = y; }

  void SetZ(const float& z) { z_ = z; }


  Vector3 operator+(const Vector3& v) {
    return Vector3(x_ + v.GetX(), y_ + v.GetY(), z_ + v.GetZ());
  }

  Vector3 operator-(const Vector3& v) {
    return Vector3(x_ - v.GetX(), y_ - v.GetY(), z_ - v.GetZ());
  }

  Vector3 operator*(float f) {
    return Vector3(f * x_, f * y_, f * z_);
  }

  Vector3 operator/(float f) {
    return Vector3(x_ / f, y_ / f, z_ / f);
  }

private:
  float x_, y_, z_;
};

inline Vector3 operator*(float f, const Vector3& v) {
  return Vector3(f * v.GetX(), f * v.GetY(), f * v.GetZ());
}

#endif  // VECTOR3_H
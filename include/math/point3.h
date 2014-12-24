#pragma once

#include <cmath>

namespace math
{
#pragma pack(push, 1)
  struct point3
  {
    double x, y, z;

    point3() : x(0), y(0), z(0) {}
    point3( double xx, double yy, double zz ) : x(xx), y(yy), z(zz) {}

    point3 operator- () const { return point3(-x, -y, -z); }

    point3& operator+= ( const point3 & rhs ) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    point3& operator-= ( const point3 & rhs ) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }

    point3& operator*= ( double s ) { x *= s; y *= s; z *= s; return *this; }
    point3& operator/= ( double s ) { x /= s; y /= s; z /= s; return *this; }

    void normalize();
  };
#pragma pack(pop)

  inline point3 operator+ ( point3 a, const point3 & b ) { return a += b; }
  inline point3 operator- ( point3 a, const point3 & b ) { return a -= b; }

  inline double dot( const point3 & a, const point3 & b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
  inline point3 cross( const point3 & a, const point3 & b ) { return point3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

  inline point3 mul( const point3 & a, const point3 & b ) { return point3(a.x * b.x, a.y * b.y, a.z * b.z); }
  inline point3 div( const point3 & a, const point3 & b ) { return point3(a.x / b.x, a.y / b.y, a.z / b.z); }

  inline point3 operator* ( point3 a, double s ) { return a *= s; }
  inline point3 operator* ( double s, point3 a ) { return a *= s; }
  inline point3 operator/ ( point3 a, double s ) { return a /= s; }

  inline double norm_sqr( const point3 & a ) { return dot(a, a); }
  inline double norm( const point3 & a ) { return sqrt(norm_sqr(a)); }

  inline double distance_sqr( const point3 & a, const point3 & b ) { return norm_sqr(a - b); }
  inline double distance( const point3 & a, const point3 & b ) { return norm(a - b); }

  inline point3 normalized( const point3 & a ) { return a / norm(a); }

  inline void point3::normalize() { *this /= norm(*this); }

  inline bool equal( const point3 & a, const point3 & b, double eps ) { return fabs(a.x - b.x) < eps && fabs(a.y - b.y) < eps && fabs(a.z - b.z) < eps; }

  inline double angle( const point3 & a, const point3 & b, double eps )
  {
      const double la_lb = sqrt(norm_sqr(a) * norm_sqr(b));
      return (la_lb < eps) ? acos(dot(a, b) / la_lb) : 0;
  }

  inline point3 bound_norm( const point3 & a, double M )
  {
    const double d = norm(a);
    if ( d > M )
      return M / d * a;

    return a;
  }
}

#pragma once

#include <cmath>

namespace math
{
#pragma pack(push, 1)
  struct point2
  {
    double x, y;

    point2() : x(0), y(0) {}
    point2( double xx, double yy ) : x(xx), y(yy) {}

    point2 operator- () const { return point2(-x, -y); }

    point2& operator+= ( const point2 & rhs ) { x += rhs.x; y += rhs.y; return *this; }
    point2& operator-= ( const point2 & rhs ) { x -= rhs.x; y -= rhs.y; return *this; }

    point2& operator*= ( double s ) { x *= s; y *= s; return *this; }
    point2& operator/= ( double s ) { x /= s; y /= s; return *this; }

    void normalize();
  };
#pragma pack(pop)

  inline point2 operator+ ( point2 a, const point2 & b ) { return a += b; }
  inline point2 operator- ( point2 a, const point2 & b ) { return a -= b; }

  inline double dot( const point2 & a, const point2 & b ) { return a.x * b.x + a.y * b.y; }
  inline double cross( const point2 & a, const point2 & b ) { return a.x * b.y - a.y * b.x; }

  inline point2 mul( const point2 & a, const point2 & b ) { return point2(a.x * b.x, a.y * b.y); }
  inline point2 div( const point2 & a, const point2 & b ) { return point2(a.x / b.x, a.y / b.y); }

  inline point2 operator* ( point2 a, double s ) { return a *= s; }
  inline point2 operator* ( double s, point2 a ) { return a *= s; }
  inline point2 operator/ ( point2 a, double s ) { return a /= s; }

  inline double norm_sqr( const point2 & a ) { return dot(a, a); }
  inline double norm( const point2 & a ) { return sqrt(norm_sqr(a)); }

  inline double distance_sqr( const point2 & a, const point2 & b ) { return norm_sqr(a - b); }
  inline double distance( const point2 & a, const point2 & b ) { return norm(a - b); }

  inline point2 perp( const point2 & a ) { return point2(-a.y, a.x); }
  inline point2 normalized( const point2 & a ) { return a / norm(a); }

  inline void point2::normalize() { *this /= norm(*this); }

  inline bool equal( const point2 & a, const point2 & b, double eps ) { return fabs(a.x - b.x) < eps && fabs(a.y - b.y) < eps; }

  inline double angle( const point2 & a, const point2 & b, double eps, double pi )
  {
      const double la_lb = sqrt(norm_sqr(a) * norm_sqr(b));
      const double ang = (la_lb > eps) ? acos(dot(a, b) / la_lb) : 0;

      return cross(a, b) < 0 ? (2 * pi - ang) : ang;
  }

  inline point2 bound_norm( const point2 & a, double M )
  {
    const double d = norm(a);
    if ( d > M )
      return M / d * a;

    return a;
  }
}

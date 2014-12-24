#pragma once

#include "point3.h"

// Rotation frame:
//   Z - up, Y - forward, X - right
//   yaw - around Z, pitch - around X, roll - around Y

namespace math
{
  struct quaternion
  {
    double w;
    point3 v;

    quaternion() : w(1), v(0, 0, 0) {}
    quaternion( double ww, const point3 & vv ) : w(ww), v(vv) {}

    quaternion operator- () const { return quaternion(-w, -v); }
    quaternion operator! () const { return quaternion(w, -v); } // conjugated, equal to inverted for union quaternion

    quaternion& operator+= ( const quaternion & rhs ) { w += rhs.w; v += rhs.v; return *this; }
    quaternion& operator-= ( const quaternion & rhs ) { w -= rhs.w; v -= rhs.v; return *this; }
    quaternion& operator*= ( const quaternion & rhs );

    quaternion& operator*= ( double s ) { w *= s; v *= s; return *this; }
    quaternion& operator/= ( double s ) { w /= s; v /= s; return *this; }

    void normalize();
  };

  inline quaternion operator+ ( quaternion a, const quaternion & b ) { return a += b; }
  inline quaternion operator- ( quaternion a, const quaternion & b ) { return a -= b; }
  inline quaternion operator* ( quaternion a, const quaternion & b ) { return a *= b; }

  inline double dot( const quaternion & a, const quaternion & b ) { return a.w * b.w + dot(a.v, b.v); }

  inline quaternion operator* ( quaternion q, double s ) { return q *= s; }
  inline quaternion operator* ( double s, quaternion q ) { return q *= s; }
  inline quaternion operator/ ( quaternion q, double s ) { return q /= s; }

  inline double norm_sqr( const quaternion & q ) { return q.w * q.w + norm_sqr(q.v); }
  inline double norm( const quaternion & q ) { return sqrt(norm_sqr(q)); }
  inline quaternion normalized( quaternion q ) { return q /= norm(q); }
  inline void quaternion::normalize() { *this /= norm(*this); }

  // rotation
  inline quaternion make_quaternion_from_axis_and_angle( const point3 & axis, double angle );
  inline quaternion make_quaternion_from_axis_angle( const point3 & axis_angle );
  inline quaternion make_quaternion_from_euler_angles( double yaw, double pitch, double roll );

  inline double get_yaw( const quaternion & q );
  inline double get_pitch( const quaternion & q );
  inline double get_roll( const quaternion & q );

  inline point3 get_axis( const quaternion & q, double eps );
  inline double get_angle( const quaternion & q );
  inline point3 get_axis_angle( const quaternion & q, double eps );

  inline quaternion rotate( const quaternion & q, const point3 & axis, double angle ) { return make_quaternion_from_axis_and_angle(axis, angle) * q; }
  inline quaternion rotate( const quaternion & q, const point3 & axis_angle ) { return make_quaternion_from_axis_angle(axis_angle) * q; }

  inline point3 rotate_point_to( const quaternion & q, const point3 & p ) { return (q * quaternion(0, p) * !q).v; }
  inline point3 rotate_point_from( const quaternion & q, const point3 & p ) { return (!q * quaternion(0, p) * q).v; }

  inline point3 get_local_omega( const quaternion & q_from, const quaternion & q_to, double dt, double eps ) { get_axis_angle(!q_from * q_to, eps) / dt; }
  inline point3 get_global_omega( const quaternion & q_from, const quaternion & q_to, double dt, double eps ) { get_axis_angle(q_to * !q_from, eps) / dt; }

  inline quaternion blend( const quaternion & q_from, quaternion q_to, double t, double eps )
  {
    double dq = dot(q_from, q_to);
    if ( dq < 0 ) // choose acute arc
    {
      dq = -dq;
      q_to = -q_to;
    }

    const double a = acos(dq);
    const double sin_a = sin(a);
    if ( fabs(sin_a) < eps )
      return q_from * (1 - t) + q_to * t;

    return q_from * sin((1 - t) * a) / sin_a + q_to * sin(t * a) / sin_a;
  }

  // implementation
  inline quaternion& quaternion::operator*= ( const quaternion & rhs )
  {
      const double A = (w   + v.x) * (rhs.w   + rhs.v.x);
      const double B = (v.z - v.y) * (rhs.v.y - rhs.v.z);
      const double C = (w   - v.x) * (rhs.v.y + rhs.v.z);
      const double D = (v.y + v.z) * (rhs.w   - rhs.v.x);
      const double E = (v.x + v.z) * (rhs.v.x + rhs.v.y);
      const double F = (v.x - v.z) * (rhs.v.x - rhs.v.y);
      const double G = (w   + v.y) * (rhs.w   - rhs.v.z);
      const double H = (w   - v.y) * (rhs.w   + rhs.v.z);

      w   = B + (-E - F + G + H) / 2;
      v.x = A - ( E + F + G + H) / 2;
      v.y = C + ( E - F + G - H) / 2;
      v.z = D + ( E - F - G + H) / 2;

      return *this;
  }

  inline quaternion make_quaternion_from_euler_angles( double yaw, double pitch, double roll )
  {
      const quaternion qz(cos(yaw / 2), point3(0, 0, sin(yaw / 2)));
      const quaternion qx(cos(pitch / 2), point3(sin(pitch / 2), 0, 0));
      const quaternion qy(cos(roll / 2), point3(0, sin(roll / 2), 0));

      quaternion r = qz * qx * qy;
      r.normalize();

      return r;
  }

  inline quaternion make_quaternion_from_axis_and_angle( const point3 & axis, double angle )
  {
    quaternion r(cos(angle / 2), axis * sin(angle / 2));
    r.normalize();

    return r;
  }
  
  inline quaternion make_quaternion_from_axis_angle( const point3 & axis_angle )
  {
    const double angle = norm(axis_angle);
    const point3 axis = axis_angle / angle;

    return make_quaternion_from_axis_and_angle(axis, angle);
  }

  inline double get_yaw( const quaternion & q )
  {
      const double dx = 1 - 2 * q.v.x * q.v.x - 2 * q.v.z * q.v.z;
      const double dy = -2 * (q.v.x * q.v.y - q.w * q.v.z);

      return atan2(dy, dx);
  }
  
  inline double get_pitch( const quaternion & q )
  {
      const double a = 2 * (q.v.y * q.v.z + q.w * q.v.x);
      return asin(a);
  }
  
  inline double get_roll( const quaternion & q )
  {
      const double dx = 1 - 2 * q.v.y * q.v.y - 2 * q.v.x * q.v.x;
      const double dy = -2 * (q.v.x * q.v.z - q.w * q.v.y);

      return atan2(dy, dx);
  }

  inline point3 get_axis( const quaternion & q, double eps )
  {
    const double l = norm(q.v);
    if ( l < eps )
      return point3(0, 0, 1); // doesn't matter, angle ~0

    return q.v / l;
  }

  inline double get_angle( const quaternion & q )
  {
    return 2 * acos(q.w);
  }

  inline point3 get_axis_angle( const quaternion & q, double eps )
  {
    return get_axis(q, eps) * get_angle(q);
  }
}

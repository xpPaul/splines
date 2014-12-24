///////////////////////////////////////////////////////////////////////////////
/// Spline builder decorators
/// All builder decorators supposed to be sealed classes
///
/// @todo implement modification of catmull_rom spline with specified flatness
/// @todo implement modification of catmull_rom spline with different tangent calculation
/// @todo implement natural cubic spline (spline which interpolate control points and provide continuously of the second derivative)
/// @todo implement closed variant of builders
/// @todo implement nurbs
/// @todo local changes on insert, delete, change ???
/// @todo move implementation to the end of file

#pragma once

#include "spline.h"

namespace gsl
{
    /// Hermite-spline cubic segment
    template < class S, class V > S hermite_segment( const V & p0, const V & p1, const V & t0, const V & t1 )
    {
        V coefs[] = { p0, t0, 3*p1 - 3*p0 - 2*t0 - t1, 2*p0 - 2*p1 + t0 + t1 };
        return S(coefs, coefs + 4);
    }

    /// Bezier cubic segment
    template < class S, class V > S bezier_segment( const V & p0, const V & p1, const V & p2, const V & p3 )
    {
        V coefs[] = { p0, 3*p1 - 3*p0, 3*p0 - 6*p1 + 3*p2, 3*p1 - p0 - 3*p2 + p3 };
        return S(coefs, coefs + 4);
    }

    /// B-Spline cubic segment
    template < class S, class V > S bspline_segment( const V & p0, const V & p1, const V & p2, const V & p3 )
    {
        V coefs[] = { p0/6 + (2/3.)*p1 + p2/6, p2/2 - p0/2, p0/2 + p2/2 - p1, p1/2 - p0/6 - p2/2 + p3/6 };
        return S(coefs, coefs + 4);
    }

    /// Bezier N-order segment
    template < class S, class InIt > S bezier_segment( InIt first, InIt last )
    {
      typedef typename S::parameter_type parameter_type;
      typedef typename S::value_type value_type;

      size_type N = std::distance(first, last) - 1;
      std::vector<value_type> coefs(N + 1, value_type());

      for ( size_type i = 0; i <= N; i++ )
      {
        parameter_type prod = 1;
        for ( size_type j = 0; j < i; j++ )
          prod *= N - j;

        value_type sum = value_type();
        for ( size_type k = 0; k <= i; k++ )
            sum += (((k + i) & 1) ? -1 : 1) * (*(first + k)) / (details::fac<parameter_type>(k) * details::fac<parameter_type>(i - k));

        coefs[i] = prod * sum;
      }

      return S(&coefs[0], &coefs[0] + N + 1);
    }

    // ----------------------------------------------------------------
    /// Bezier cubic spline. To provide first derivative continuously 3 points (i-1, i, i+1) near segments connection should be collinear
    template < class Base >
        class bezier_spline
    {
    protected:
        template < class Pts, class OutIt > static void build( const Pts & pts, size_type from, size_type to, OutIt out )
        {
            typedef typename Base::value_type value_type;
            typedef typename Base::segment_type segment_type;

            for ( size_type i = from; i + 3 < to; i += 3 )
            {
                value_type p0 = pts[i+0];
                value_type p1 = pts[i+1];
                value_type p2 = pts[i+2];
                value_type p3 = pts[i+3];

                *out++ = bezier_segment<segment_type>(p0, p1, p2, p3);
            }
        }
    };

    // ----------------------------------------------------------------
    /// Bezier N-order spline
    template < size_t N, class Base = void >
        class bezier_n_spline
    {
    public:
        // Convert to BuildPolicy interface
        template < class T > struct apply : bezier_n_spline<N, T>
        {
            typedef bezier_n_spline<N, T> type;
        };

    protected:
        template < class Pts, class OutIt > static void build( const Pts & pts, size_type from, size_type to, OutIt out )
        {
            typedef typename Base::segment_type segment_type;
            for ( size_type i = from; i + N < to; i += N )
                *out++ = bezier_segment<segment_type>(&pts[0] + i, &pts[0] + i + N + 1);
        }
    };

    // ----------------------------------------------------------------
    /// Catmull-Rom spline (Hermite spline with p'(i) = p(i+1) - p(i-1))
    /// Provide continuously of the first derivative
    template < class Base >
        class catmull_rom_spline
    {
    protected:
        template < class Pts, class OutIt > static void build( const Pts & pts, size_type from, size_type to, OutIt out )
        {
            typedef typename Base::value_type value_type;
            typedef typename Base::segment_type segment_type;

            for ( size_type i = from; i + 1 < to; i++ )
            {
                value_type p0 = (i > 0) ? pts[i - 1] : pts.front();
                value_type p1 = pts[i];
                value_type p2 = pts[i + 1];
                value_type p3 = (i + 2 < pts.size()) ? pts[i + 2] : pts.back();

                value_type t1 = (p2 - p0) / 2;
                value_type t2 = (p3 - p1) / 2;

                *out++ = hermite_segment<segment_type>(p1, p2, t1, t2);
            }
        }
    };

    // ----------------------------------------------------------------
    /// B-Spline. Do not pass all control points. Provide continuously of the second derivative
    template < class Base >
        class b_spline
    {
    protected:
        template < class Pts, class OutIt > static void build( const Pts & pts, size_type from, size_type to, OutIt out )
        {
            typedef typename Base::value_type value_type;
            typedef typename Base::segment_type segment_type;

            for ( size_type i = from; i + 1 < to; i++ )
            {
                value_type p0 = (i > 0) ? pts[i - 1] : (2*pts[0]-pts[1]);
                value_type p1 = pts[i];
                value_type p2 = pts[i + 1];
                value_type p3 = (i + 2 < pts.size()) ? pts[i + 2] : (2*pts.back() - pts[pts.size()-2]);

                *out++ = bspline_segment<segment_type>(p0, p1, p2, p3);
            }
        }
    };

    // ----------------------------------------------------------------
    /// spline_builder class template
    template < class Base, template <class> class BuildPolicy >
        class spline_builder
            : public Base
            , public BuildPolicy<Base>
    {
    public:
        //@{ Common types definition
        typedef typename Base::segment_type segment_type;
        typedef typename Base::parameter_type parameter_type;
        typedef typename Base::value_type value_type;
        //@}

    public:
        /// Default constructor
        spline_builder() {}

        /// Constructor by control points
        template < class InIt >
            spline_builder( InIt first, InIt last )
                : m_ControlValues(first, last)
        {
            std::vector<segment_type> segs;
            this->build(m_ControlValues, 0, m_ControlValues.size(), back_inserter(segs));
            this->assign(segs.begin(), segs.end());
        }

        /// Control points direct access
        const std::vector<value_type> & control_values() const { return m_ControlValues; }

        /// @todo rebuild locally and use Base::replace in remove and insert methods
        
        /// Remove control point and rebuild spline
        void remove( size_type where )
        {
            m_ControlValues.erase(m_ControlValues.begin() + where);

            std::vector<segment_type> segs;
            this->build(m_ControlValues, 0, m_ControlValues.size(), back_inserter(segs));
            this->assign(segs.begin(), segs.end());
        }

        /// Insert new control point and rebuild spline
        void insert( size_type where, value_type val )
        {
            m_ControlValues.insert(m_ControlValues.begin() + where, val);

            std::vector<segment_type> segs;
            this->build(m_ControlValues, 0, m_ControlValues.size(), back_inserter(segs));
            this->assign(segs.begin(), segs.end());
        }
        
        /// Change existing control point value
        void change( size_type where, value_type val )
        {
            m_ControlValues.at(where) = val;

            std::vector<segment_type> segs;
            this->build(m_ControlValues, 0, m_ControlValues.size(), back_inserter(segs));
            this->assign(segs.begin(), segs.end());
        }

    private:
        std::vector<value_type> m_ControlValues;
    };
}

///////////////////////////////////////////////////////////////////////////////
/// segment class template definition

#pragma once

#include <utility>
#include <algorithm>
#include <numeric>

#include "splines_aux.h"

namespace gsl
{
    // ----------------------------------------------------------------
    /// segment class template
    ///     Performs interpolation in range [0, 1]
    ///     If parameter is out of range value will be extrapolated
    template < typename T, typename U, size_type D >
        class segment
    {
    public:
        static const size_type Degree = D; ///< Segment degree (0 - const, 1 - linear, 3 cubic, ...)

        typedef T parameter_type; ///< Some real type expected to be parameter type
        typedef U value_type; ///< Some vector value expected

    public:
        /// Default constructor
        segment();

        /// Constructor by coefficients
        template < typename InIt >
            segment( InIt first, InIt last );

        /// Obtain origin = s(0) and ending = s(1) values
        value_type origin() const;
        value_type ending() const;

        /// Obtain interpolated value
        value_type operator() ( parameter_type t ) const;

        /// Obtain 'K' derivative value
        template < size_type K > 
            value_type derivative( parameter_type t ) const;

        /// Approximate segment with polyline
        template < class OutPtIt >
            void approximate ( parameter_type accuracy, OutPtIt out ) const;

        /// Obtain curvature
        parameter_type curvature( parameter_type t ) const;

        /// Obtain curvature radius
        parameter_type radius( parameter_type t ) const;

        /// Obtain torsion
        parameter_type torsion( parameter_type t ) const;

        /// Obtain torsion radius
        parameter_type torsion_radius( parameter_type t ) const;

        /// Obtain direction
        value_type direction( parameter_type t ) const;

        /// Obtain normal
        value_type normal( parameter_type t ) const;

        /// Obtain binormal
        value_type binormal( parameter_type t ) const;

    private:
        /// Approximate segment with polyline
        template < class OutPtIt >
            void approximate( parameter_type t0, parameter_type t1, const value_type & p0, const value_type & p1, parameter_type accuracy, OutPtIt out ) const;

    protected:
        value_type m_Coefs[Degree + 1];
    };

    // ================================================================
    // segment class template
    // Implementation

#define TE template < typename T, typename U, size_type D >
#define ME segment<T, U, D>::

    // ----------------------------------------------------------------
    TE ME segment()
    {
        std::fill_n(m_Coefs, Degree + 1, U());
    }

    // ----------------------------------------------------------------
    TE template < typename InIt > ME segment( InIt first, InIt last )
    {
        //std::copy(first, std::min(last, first + Degree + 1), m_Coefs);
        std::copy(first, last, m_Coefs);
    }

    // ----------------------------------------------------------------
    TE U ME origin() const
    {
      return m_Coefs[0];
    }

    // ----------------------------------------------------------------
    TE U ME ending() const
    {
      return std::accumulate(m_Coefs, m_Coefs + Degree + 1, U());
    }

    // ----------------------------------------------------------------
    TE U ME operator() ( T t ) const
    {
        T tt = t;
        U ret = m_Coefs[0];

        for ( size_type i = 1; i <= Degree; i++, tt *= t )
            ret += tt * m_Coefs[i];

        return ret;
    }

    // ----------------------------------------------------------------
    // (a0 + a1*t + a2 * t^2 + a3 * t^3 + ... + an * t^n)'k =
    //      = k! * ak + (k+1)!/2 + ... + n!/(n-k)! * an * t^(n-k)
    TE template < size_type Deg >
        U ME derivative( T t ) const
    {
        T tt = 1;
        U ret = U();

        for ( size_type i = Deg; i <= Degree; i++, tt *= t )
        {
            T coef = details::d_coef(i, Deg);
            ret += (tt * coef) * m_Coefs[i];
        }

        return ret;
    }

    // ----------------------------------------------------------------
    // Obtain curvature
    /// @todo check
    TE T ME curvature( T t ) const
    {
      U d1 = this->derivative<1>(t);
      U d2 = this->derivative<2>(t);
      T nd1 = details::norm_(d1);
      return details::norm_(cross(d1, d2)) / (nd1 * nd1 * nd1);
    }

    // Obtain curvature radius
    TE T ME radius( T t ) const
    {
        U d1 = this->derivative<1>(t);
        U d2 = this->derivative<2>(t);
        T nd1 = details::norm_(d1);
        return (nd1 * nd1 * nd1) / details::norm_(cross(d1, d2));
    }

    /// Obtain torsion
    TE T ME torsion( T t ) const
    {
        return 0; // todo: implement
    }

    /// Obtain torsion radius
    TE T ME torsion_radius( T t ) const
    {
        return 1 / this->torsion(t);
    }

    /// Obtain direction
    /// Use normalized first derivative or try next derivative if previous is zero
    template < size_type N > struct obtain_direction
    {
      TE static U apply( const segment<T, U, D> & s, T t )
      {
        U dir = s.template derivative<D - N>(t);
        T l = details::norm_(dir);

        return details::eq_zero(l) ? obtain_direction<N - 1>::apply(s, t) : details::normalized_(dir);
      }
    };
    template <> struct obtain_direction<0>
    {
      TE static U apply( const segment<T, U, D> & s, T )
      {
        return s.ending() - s.origin();
      }
    };

    TE U ME direction( T t ) const
    {
        return obtain_direction<D - 1>::apply(*this, t);
    }

    /// Obtain normal
    TE U ME normal( T t ) const
    {
        return details::perp_(this->direction(t)); // todo: implement
    }

    /// Obtain binormal
    TE U ME binormal( T t ) const
    {
        return this->direction(t) ^ this->normal(t);
    }

    /// Approximate segment with polyline
    TE template < class OutPtIt > void ME approximate ( T accuracy, OutPtIt out ) const
    {
      /// @todo nonrecursive 
      const size_type NSubDiv = 5;
      
      T t0 = 0;
      U p0 = (*this)(t0);
      
      for ( size_type i = 1; i <= NSubDiv; i++ )
      {
        T t1 = i / T(NSubDiv);
        U p1 = (*this)(t1);
        this->approximate(t0, t1, p0, p1, accuracy, out);
        p0 = p1;
        t0 = t1;
      }

      *out++ = p0;
    }

    /// Approximate segment with polyline
    TE template < class OutPtIt > void ME approximate( T t0, T t1, const U & p0, const U & p1, T accuracy, OutPtIt out ) const
    {
      T t = (t0 + t1) / 2;
      U p = (*this)(t);
      U pc = (p0 + p1) / 2; /// @todo: use distance(line_segment(p0,p1), p) instead, use double-checked estimate

      if ( details::norm_(p - pc) < accuracy )
      {
        *out++ = p0;
        return;
      }

      this->approximate(t0, t, p0, p, accuracy, out);
      this->approximate(t, t1, p, p1, accuracy, out);
    }


#undef TE
#undef ME

}

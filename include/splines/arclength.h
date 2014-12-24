///////////////////////////////////////////////////////////////////////////////
/// arclength parametrization for segment and spline
/// arclen(t0, t1) = integral[t0, t1]( |s'(t)| )dt
/// t2s(t) = arclen(0, t)
/// s2t(s) = t : (s - t2s(t)) == 0

#pragma once

#include "segment.h"
#include "spline.h"

namespace gsl
{
    // ----------------------------------------------------------------
    /// segment arclength parametrization class template, compile-time decorator for segment
    ///      This class supposed that U - point in Euclidian space, T - real type
    ///      This class implements segment arc-length parametrization with required accuracy
    template < class Base >
        class segment_arclength
            : public Base
    {
    public:
        GSL_SEGMENT_DECORATOR(segment_arclength);

    public:
        /// Get arc-length of specified part of segment
        parameter_type length( parameter_type from, parameter_type to, parameter_type accuracy ) const;
        
        /// Get full length of segment
        parameter_type length( parameter_type accuracy ) const { return length(0, 1, accuracy); }

        /// Convert parameter to natural parameter
        parameter_type t2s( parameter_type t, parameter_type accuracy ) const;

        /// Convert natural parameter to original [0, 1] parameter
        parameter_type s2t( parameter_type s, parameter_type accuracy ) const;
    };

    // ----------------------------------------------------------------
    /// spline parametrizator class template, compile-time decorator for spline
    template < class Base, class S = segment_arclength<typename Base::segment_type> >
        class spline_arclength
            : public Base::template apply<S>::type
    {
    public:
        GSL_SPLINE_DECORATOR(spline_arclength);

    public:
        /// Set required accuracy (of parameter 't')
        void set_parametrization_accuracy( parameter_type accuracy ) { m_Accuracy = accuracy; }

        /// Get full spline length
        parameter_type length() const { return t2s(this->size()); }

        /// Convert parameter to natural parameter
        parameter_type t2s( parameter_type t ) const;

        /// Convert natural parameter to original [0, 1] parameter
        parameter_type s2t( parameter_type s ) const;

    private:
        parameter_type m_Accuracy;
    };


    // ================================================================
    // segment_arclength class template
    // Implementation

#define TE template < class Base >
#define ME segment_arclength<Base>::

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME length( parameter_type from, parameter_type to, parameter_type accuracy ) const
    {
        // TODO: clip [to, from] to [0, 1] ???

        parameter_type c = (from + to) / 2;

        value_type p0 = (*this)(from);
        value_type p1 = (*this)(c);
        value_type p2 = (*this)(to);

        parameter_type l0 = details::norm_(p0 - p2);
        parameter_type l1 = details::norm_(p0 - p1) + details::norm_(p1 - p2);

        if ( l1 - l0 < accuracy ) // first estimate
        {
          value_type p01 = (*this)((from + c) / 2);
          value_type p12 = (*this)((c + to) / 2);

          parameter_type l2 = details::norm_(p0 - p01) + details::norm_(p01 - p1) + details::norm_(p1 - p12) + details::norm_(p12 - p2);

          if ( l2 - l0 < accuracy ) // second estimate
              return (16 * l2 - l1) / 15;
        }
        
        // Single estimate
        //if ( l1 - l0 < accuracy )
        //    return (16 * l1 - l0) / 15;

        return this->length(from, c, accuracy) + this->length(c, to, accuracy);
    }

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME t2s( parameter_type t, parameter_type accuracy ) const
    {
        return this->length(0, std::min(parameter_type(1), std::max(parameter_type(0), t)), accuracy);
    }

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME s2t( parameter_type s, parameter_type accuracy ) const
    {
        parameter_type step = 1, t = 1;
        parameter_type l = this->length(accuracy);
        
        do
        {
            parameter_type d = this->t2s(t, accuracy) - s;

            if ( fabs(d) < accuracy )
                break;

            if ( d < 0 )
                t += step;
            else
                t -= step;

            step /= 2;
        }
        while ( t <= 1 && 0 <= t && l * step > accuracy );

        if ( fabs(this->t2s(t, accuracy) - s) > accuracy )
            t = t; // binary search failed problem! (проблема может возникать из-за точек перегиба)

        return std::min(parameter_type(1), std::max(parameter_type(0), t));
    }

#undef TE
#undef ME

    // ================================================================
    // spline_arclength class template
    // Implementation

#define TE template < class Base, class S >
#define ME spline_arclength<Base, S>::

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME t2s( parameter_type t ) const
    {
        if ( this->empty() )
            return 0;

        size_type idx = this->parameter2idx(t);

        parameter_type s = (*this)[idx].t2s(t, m_Accuracy);
        for ( size_type i = 0; i < idx; i++ )
          s += (*this)[i].length(m_Accuracy / this->size());
          //s += (*this)[i].length(m_Accuracy); <-- error accumulation


        return s;
    }
    
    // ----------------------------------------------------------------
    TE typename ME parameter_type ME s2t( parameter_type s ) const
    {
        for ( size_t idx = 0; idx < this->size(); idx++ )
        {
            const parameter_type seg_length = (*this)[idx].length(m_Accuracy);
            if ( s > seg_length )
                s -= seg_length;
            else
                return idx + (*this)[idx].s2t(s, m_Accuracy);
        }

        return this->size() + 1;
    }

#undef TE
#undef ME

}

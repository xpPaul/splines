///////////////////////////////////////////////////////////////////////////////
/// localization abilities for segment and spline
///
/// @todo implement get_aabb method
/// @todo implement get_hull method for 2-dimensional value_type
/// @todo implement has_intersection, intersect methods for 2-dimensional value_type
/// @todo implement grid or aabb-tree based localization for 2-dimensional value_type

#pragma once

#include <functional>

#include "segment.h"
#include "spline.h"

namespace gsl
{
    // ----------------------------------------------------------------
    /// segment localization class template, compile-time decorator for segment
    ///      This class supposed that U - point in Euclidean space, T - real type
    ///      This class implements segment localization requests with required accuracy
    template < class Base >
        class segment_localization
            : public Base
    {
    public:
        GSL_SEGMENT_DECORATOR(segment_localization);

    public:
        /// Obtain distance from specified point to spline and parameter of the closest point on segment
        parameter_type distance ( value_type p, parameter_type * t, parameter_type accuracy ) const;

        void get_aabb( value_type * min, value_type * max ) const;

        template < class OutIt > void get_hull( OutIt out ) const;

        bool intersect( value_type s0, value_type s1, parameter_type * t ) const;
    };

    // ----------------------------------------------------------------
    /// spline localization class template, compile-time decorator for spline
    // ----------------------------------------------------------------
    template < class Base, class S = segment_localization<typename Base::segment_type> >
        class spline_localization
            : public Base::template apply<S>::type
    {
    public:
        GSL_SPLINE_DECORATOR(spline_localization);

    public:
        /// Set required accuracy (of parameter 't' obtained in distance) (current implementation not guarantee this in some special cases)
        void set_localization_accuracy( parameter_type accuracy ) { m_Accuracy = accuracy; }

        /// Same as segment_localization::distance, but for spline
        parameter_type distance( value_type p, parameter_type * t = 0 ) const;

        /// Same as segment_localization::intersect, but for spline
        bool intersect( value_type s0, value_type s1, parameter_type * t ) const;

    private:
        parameter_type m_Accuracy;
    };


    // ================================================================
    // segment_localization class template
    // Implementation

#define TE template < class Base >
#define ME segment_localization<Base>::

    namespace details
    {
    template < class segment_type >
    struct point_to_segment_distance_t // <=> bind(details::distance_, p, bind(&Base::operator(), this, _1))
    {
        typedef typename segment_type::parameter_type parameter_type;
        typedef typename segment_type::value_type value_type;

        point_to_segment_distance_t( const value_type & p, const segment_type & s ) : p_(p), s_(s) {}

        parameter_type operator() ( parameter_type t ) const
        {
            return details::norm_(p_ - s_(t));
        }

    private:
        const value_type & p_;
        const segment_type & s_;
    };
    }

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME distance ( value_type p, parameter_type * t, parameter_type accuracy ) const
    {
        /// @todo initial subdivision (проблема с точками перегиба)
        /// @todo distance -> distance_sqr

        parameter_type tc = details::golden_section(parameter_type(0), parameter_type(1), accuracy,
                                                    details::point_to_segment_distance_t<Base>(p, *this));

        if ( t )
            *t = tc;

        return details::norm_(p - (*this)(tc));
    }

#undef TE
#undef ME

    // ================================================================
    // spline_localization class template
    // Implementation

#define TE template < class Base, class S >
#define ME spline_localization<Base, S>::

    // ----------------------------------------------------------------
    TE typename ME parameter_type ME distance ( value_type p, parameter_type * t ) const
    {
        parameter_type mt = 0, md = -1;
        for ( size_type i = 0; i < this->size(); i++ )
        {
            parameter_type t;
            parameter_type d = (*this)[i].distance(p, &t, m_Accuracy);
            if ( d < md || md == -1 )
            {
                md = d;
                mt = i + t;
            }
        }

        if ( t )
            *t = mt;

        return md;
    }

    // ----------------------------------------------------------------
    TE bool ME intersect( value_type s0, value_type s1, parameter_type * t ) const
    {
        parameter_type mt = 0, md = -1;
        for ( size_type i = 0; i < this->size(); i++ )
        {
            parameter_type t;
            if ( (*this)[i].intersect(s0, s1, &t) )
            {
                parameter_type d = details::norm_(s0 - (*this)[i](t));
                if ( d < md || md == -1 )
                {
                    md = d;
                    mt = i + t;
                }
            }
        }

        if ( t )
            *t = mt;

        return md != -1;
    }

#undef TE
#undef ME

}

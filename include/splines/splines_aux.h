///////////////////////////////////////////////////////////////////////////////
/// common definitions and auxiliary stuff

#pragma once

#include <stdexcept>

namespace gsl
{
    typedef size_t size_type;

    // ----------------------------------------------------------------
    /// gsl exceptions base class
    class exception : public std::runtime_error
    {
    public:
        explicit exception( const std::string & msg ) : std::runtime_error(msg) {}
    };

    // ----------------------------------------------------------------
    /// Derived segment decorators should use this macros to provide constructing ability
#define GSL_SEGMENT_DECORATOR(decorator)        \
    decorator () {}                     \
    template < class Seg > decorator( const Seg & rhs ) : Base(rhs) {}                                  \
    template < class Seg > decorator& operator= ( const Seg & rhs ) { return *this = decorator(rhs); }  \
    template < typename InIt > decorator ( InIt first, InIt last ) : Base(first, last) {}               \
    typedef typename Base::parameter_type parameter_type;                                               \
    typedef typename Base::value_type value_type;

    // ----------------------------------------------------------------
    /// Derived spline decorators should use this macros to provide constructing ability
#define GSL_SPLINE_DECORATOR(decorator)     \
    decorator () {}                         \
    template < typename InIt > decorator( InIt first, InIt last ) : Base::template apply<S>::type(first, last) {} \
    template < typename OtherS > struct apply { typedef spline_arclength<Base, OtherS> type; };                 \
    template < class OtherS > decorator( const OtherS & rhs ) : Base::template apply<S>::type(rhs.begin(), rhs.end()) {} \
    template < class OtherS > decorator& operator= ( const OtherS & rhs ) { return *this = decorator(rhs); }    \
    typedef S segment_type;                                                                                     \
    typedef typename segment_type::parameter_type parameter_type;                                               \
    typedef typename segment_type::value_type value_type;

    // ----------------------------------------------------------------
    namespace details
    {
        inline size_type d_coef( size_type n, size_type k )
        {
            if ( k > n ) return 0;
            if ( k == 0 ) return 1;

            size_t ret = n;

            while ( --k )
                ret *= --n;

            return ret;
        }

        template < class T > T fac( size_type n )
        {
            T res = 1;
            for ( size_type i = 2; i <= n; i++ )
                res *= i;

            return res;
        }

        template < class T, class Fn > T golden_section( T a, T b, T accuracy, Fn f )
        {
            T s1 = (3 - sqrt(5.)) / 2;
            T s2 = (sqrt(5.) - 1) / 2;
            T u1 = a + s1 * (b - a);
            T u2 = a + s2 * (b - a);
            T fu1 = f(u1);
            T fu2 = f(u2);

            while ( fabs(a - b) > accuracy )
            {
                if ( fu1 <= fu2 )
                {
                    b   = u2;
                    u2  = u1;
                    fu2 = fu1;
                    u1  = a + s1 * (b - a);
                    fu1 = f(u1);
                }
                else
                {
                    a   = u1;
                    u1  = u2;
                    fu1 = fu2;
                    u2  = a + s2 * (b - a);
                    fu2 = f(u2);
                }
            }

            return (fu1 < fu2) ? a : b;
        }

        // @todo refactor this
        inline double norm_( double p )
        {
            return p;
        }

        inline bool eq_zero( double x )
        {
            return x < 1e-8;
        }

        // @todo move to value_type_traits ???
        template < class value_type >
        inline double norm_( const value_type & p )
        {
            return norm(p);
        }

        template < class value_type >
        inline value_type perp_( const value_type & dir )
        {
           return perp(dir);
        }

        template < class value_type >
        inline value_type normalized_( const value_type & p )
        {
            return normalized(p);
        }
    }
}

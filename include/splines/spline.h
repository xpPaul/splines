///////////////////////////////////////////////////////////////////////////////
/// spline class template definition

#pragma once

#include <vector>
#include <string>

#include "segment.h"

namespace gsl
{
    // ----------------------------------------------------------------
    /// This struct check are values equal or not
    template < typename U >
        struct segments_connected_verification_traits
    {
        static bool eq( U a, U b ) { return a == b; }
    };

    // ----------------------------------------------------------------
    /// This exception will be thrown if spline invariant breaked
    class spline_segments_disconnected_exception : public exception
    {
    public:
        explicit spline_segments_disconnected_exception( const std::string & msg ) : exception(msg) {}
    };

    class spline_empty_exception : public exception
    {
    public:
      explicit spline_empty_exception( const std::string & msg ) : exception(msg) {}
    };

    // ----------------------------------------------------------------
    /// spline class template
    ///      Contains N segments
    ///      Performs interpolation in range [0, N]
    ///      If parameter is out of range value will be truncated.
    ///
    /// Class invariant: s[i+1](1) == s[i](0);
    /// If modifiers will try to break invariant then 'spline_segments_disconnected_exception' will be thrown
    template < typename S, class SCVT = segments_connected_verification_traits<typename S::value_type> >
        class spline
    {
    public:
        /// Apply metafunction provide the ability to obtain spline with other other segment type
        /// It's required to use decorated segments in spline decorators
        template < typename OtherS > struct apply
        {
            typedef spline<OtherS, SCVT> type;
        };

    public:
        static const size_type Degree = S::Degree;

        //@{ common types definition
        typedef SCVT segments_connected_verification_traits;
        typedef S segment_type;

        typedef typename segment_type::parameter_type parameter_type;
        typedef typename segment_type::value_type value_type;

        typedef typename std::vector<segment_type>::const_iterator const_iterator;
        typedef typename std::vector<segment_type>::const_reference const_reference;
        //@}

    public:
        /// Default constructor
        spline () {}

        /// Generic copy constructor
        template < class OtherS >
            spline ( const OtherS & rhs )
        {
            assign(rhs.begin(), rhs.end());
        }

        /// Generic assignment operator
        template < class OtherS >
            spline & operator= ( const OtherS & rhs )
        {
            return *this = spline(rhs); 
        }

        /// Constructor by segments
        template < typename InIt >
            spline ( InIt first, InIt last );

        /// Obtain interpolated value
        value_type operator() ( parameter_type t ) const;

        /// Obtain 'K' derivative value
        template < size_type K >
            value_type derivative ( parameter_type t ) const;

        /// Number of spline segments
        size_type size () const { return m_Segs.size(); }

        /// Check spline is empty
        bool empty () const { return m_Segs.empty(); }
        
        /// Access to the segment with specified index
        const_reference operator[] ( size_type idx ) const { return m_Segs[idx]; }

        /// Checked access to the segment with specified index
        const_reference at ( size_type idx ) const { return m_Segs.at(idx); }

        /// Iterator which points to the first segment
        const_iterator begin () const { return m_Segs.begin(); }

        /// Iterator which points to the element after last
        const_iterator end () const { return m_Segs.end(); }

        /// Remove segments from 'from' to 'to' and insert segments [first, last) instead
        template < class InIt >
            void replace ( size_type from, size_type to, InIt first, InIt last );

        /// Replace all existing segments with specified [first, last) segments
        template < typename InIt >
            void assign ( InIt first, InIt last );

        /// Swap two splines
        void swap ( spline & rhs ) { m_Segs.swap(rhs.m_Segs); }
        
        /// Clear spline
        void clear () { m_Segs.clear(); }

        /// Approximate spline with polyline
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

    protected:
        void verify() const;
        size_type parameter2idx( parameter_type & t ) const;

    private:
        std::vector<segment_type> m_Segs;
    };

    // ================================================================
    // spline class template
    // Implementation

#define TE template < typename S, class SCVT >
#define ME spline<S, SCVT>::

    // ----------------------------------------------------------------
    TE typename S::value_type ME operator() ( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i](t);
    }

    // ----------------------------------------------------------------
    TE template < size_type Deg > typename S::value_type ME derivative ( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].derivative<Deg>(t);
    }

    // ----------------------------------------------------------------
    TE template < class InIt > ME spline ( InIt first, InIt last )
        : m_Segs(first, last)
    {
        this->verify();
    }

    // ----------------------------------------------------------------
    TE template < class InIt > void ME replace ( size_type from, size_type to, InIt first, InIt last )
    {
        typename std::vector<segment_type>::iterator it = m_Segs.erase(m_Segs.begin() + from, m_Segs.begin() + to);
        m_Segs.insert(it, first, last);

        // TODO: verify(from, to) method
        this->verify();
    }

    // ----------------------------------------------------------------
    TE template < class InIt > void ME assign ( InIt first, InIt last )
    {
        spline tmp(first, last);
        tmp.swap(*this);
    }

    // ----------------------------------------------------------------
    TE template < class OutPtIt > void ME approximate ( parameter_type accuracy, OutPtIt out ) const
    {
      for ( size_type i = 0; i < m_Segs.size(); i++ )
        m_Segs[i].approximate(accuracy, out);
    }

    // ----------------------------------------------------------------
    TE typename S::parameter_type ME curvature( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].curvature(t);
    }

    // ----------------------------------------------------------------
    TE typename S::parameter_type ME radius( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].radius(t);
    }

    // ----------------------------------------------------------------
    TE typename S::parameter_type ME torsion( parameter_type t ) const
    {
        if ( m_Segs.empty() )
            return parameter_type(); // TODO: throw exception ?

        size_type i = this->parameter2idx(t);
        return m_Segs[i].torsion(t);
    }

    // ----------------------------------------------------------------
    TE typename S::parameter_type ME torsion_radius( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].torsion_radius(t);
    }

    // ----------------------------------------------------------------
    TE typename S::value_type ME direction( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].direction(t);
    }

    // ----------------------------------------------------------------
    TE typename S::value_type ME normal( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].normal(t);
    }

    // ----------------------------------------------------------------
    TE typename S::value_type ME binormal( parameter_type t ) const
    {
        size_type i = this->parameter2idx(t);
        return m_Segs[i].binormal(t);
    }

    // ----------------------------------------------------------------
    TE void ME verify() const
    {
      for ( size_type i = 0; i + 1 < m_Segs.size(); i++ )
          if ( !SCVT::eq(m_Segs[i](1), m_Segs[i + 1](0)) )
              throw spline_segments_disconnected_exception("");
    }

    // ----------------------------------------------------------------
    TE size_type ME parameter2idx( parameter_type & t ) const
    {
        if ( m_Segs.empty() )
            throw spline_empty_exception("");

        if ( t <= 0 )
        {
            t = 0;
            return 0;
        }
        
        if ( t >= m_Segs.size() )
        {
            t = 1;
            return m_Segs.size() - 1;
        }

        size_type i = size_type(t);
        t -= i;

        /// @todo think about extrapolation behavior customization with traits - clip, cycle, ...
        return std::max(size_type(0), std::min(size_type(m_Segs.size()), i));
    }

#undef TE
#undef ME

}

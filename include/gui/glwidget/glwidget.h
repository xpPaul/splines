#pragma once

#include <QtOpenGL>
#include <QtGui/QMouseEvent>

#include "include/math/point2.h"
using math::point2;

// QGLWidgetEx = QGLWidget + virtual 2d space + gl* wrappers
class QGLWidgetEx : public QGLWidget
{
   Q_OBJECT

public:
   struct IEvent
   {
       virtual void OnInit   () = 0;
       virtual void OnDraw   () = 0;
       virtual void OnMove   ( bool * redraw ) = 0;
       virtual bool OnButton ( bool left, bool down, bool * redraw ) = 0;
       virtual void OnDblClk ( bool left, bool * redraw ) = 0;
       virtual bool OnKey    ( int vkey, bool down, bool * redraw ) = 0;
   };

   enum AlignFlags
   {
      AlignLeft = 1, AlignCenter = 2, AlignRight = 4,
      AlignBottom = 8, AlignHCenter = 16, AlignTop = 32
   };

public:
   QGLWidgetEx();
   ~QGLWidgetEx();

   void Advise( IEvent * event );
   void Unadvise();

   void RePaint();

   void SetGlobalView( const point2 & center, double ratio );
   void GetGlobalView( point2 * center, double * ratio );

   QPoint GetCursorPos();
   point2 GetCursorPosGlobal();

   point2 ToGlobal( const QPoint & pt );
   QPoint ToScreen( const point2 & pt );

   void SetColor( double r, double g, double b, double a = 1 ); // todo: rai wrapper color_setter
   void SetLineWidth( double width ); // todo: rai wrapper line_width_setter

   void Point     ( const point2 & pt );
   void Line      ( const point2 & pt0, const point2 & pt1 );
   void Circle    ( const point2 & center, double radius, bool fill = false, size_t num_subdiv = 20 ); // num_subdiv == 0 -> autodetect
   void Triangle  ( const point2 & pt0, const point2 & pt1, const point2 & pt2, bool fill = false );
   void Rectangle ( const point2 & pt0, const point2 & pt1, bool fill = false );

   void Points    ( const point2 * points, size_t n );
   void Polyline  ( const point2 * points, size_t n );

   int NewDisplayList();
   void EndDisplayList(); // todo: raii wrapper display_list_creator
   void CallDisplayList( int id );
   void DeleteDisplayList( int id ); // todo: raii wrapper display_list

   void PutText( const point2 & pt, const QString & text, int align = AlignLeft|AlignBottom ); // todo: add scale
   void PutText( const QPoint & pt, const QString & text, int align = AlignLeft|AlignBottom );
   QSize GetTextExtent( const QString & text );

protected:
   void initializeGL();
   void paintGL();
   void resizeGL( int width, int height );
   void mouseMoveEvent( QMouseEvent *event );
   void mousePressEvent( QMouseEvent *event );
   void mouseReleaseEvent( QMouseEvent *event );
   void mouseDoubleClickEvent( QMouseEvent *event );
   void wheelEvent( QWheelEvent *event );
   void keyPressEvent( QKeyEvent * event );
   void keyReleaseEvent( QKeyEvent * event );

private:
   bool DoMouseButton( QMouseEvent * event, bool down );
   bool DoKey( QKeyEvent * event, bool down );
   void ApplySize();

private:
   IEvent * event_handler_; // todo: singals/slots

   QPoint cursor_;

   point2 center_;
   double ratio_;
};

inline double clamp( double x, double x0, double x1, double y0, double y1 )
{
   return (x < x0) ? y0 : (x > x1) ? y1 : (y0 + (y1 - y0) * (x - x0) / (x1 - x0));
}

inline void draw_plot( QGLWidgetEx & drawer, const std::vector<double> & vals, double val_min, double val_max, const point2 & p0, const point2 & p1 )
{
   point2 p_prev;
   for ( size_t i = 0; i < vals.size(); i++ )
   {
      const double val01 = clamp(vals[i], val_min, val_max, 0., 1.);
      const point2 p(p0.x + i / double(vals.size() - 1) * (p1.x - p0.x), p0.y + val01 * (p1.y - p0.y));

      if ( i != 0 )
         drawer.Line(p_prev, p);

      p_prev = p;
   }
}

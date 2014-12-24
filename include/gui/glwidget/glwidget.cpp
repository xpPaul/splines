#include "glwidget.h"

static inline double lim ( double x )
{
    if ( x < 0 ) return 0;
    if ( x > 1 ) return 1;
    return x;
}

QGLWidgetEx::QGLWidgetEx()
    : event_handler_(0)
    , center_(0, 0)
    , ratio_(1)
{
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
}

QGLWidgetEx::~QGLWidgetEx()
{}

void QGLWidgetEx::initializeGL()
{
    cursor_ = mapFromGlobal(QCursor::pos());

    qglClearColor(QColor(0, 0, 0));

    glLineWidth(1);
    glPointSize(1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    //glDisable(GL_BLEND);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA,  GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_LINE_SMOOTH); // faster
    //glEnable(GL_LINE_SMOOTH);
    //glDisable(GL_POINT_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glClearDepth(1.);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    setFocus();

    if ( event_handler_ )
        event_handler_->OnInit();
}

void QGLWidgetEx::paintGL()
{
    SetColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    SetColor(1, 0, 0);
    Line(point2(), point2(1, 0));
    SetColor(0, 1, 0);
    Line(point2(), point2(0, 1));

    if ( event_handler_ )
        event_handler_->OnDraw();
}

void QGLWidgetEx::resizeGL(int , int )
{
    ApplySize();
}

void QGLWidgetEx::mouseMoveEvent(QMouseEvent *event)
{
    const point2 p_prev = ToGlobal(cursor_);
    cursor_ = event->pos();

    if ( event->buttons() & Qt::MiddleButton )
    {
        center_.x = p_prev.x - (cursor_.x() - width()  / 2.) * ratio_;
        center_.y = p_prev.y + (cursor_.y() - height() / 2.) * ratio_;

        ApplySize();
    }

    if ( event_handler_ )
    {
        bool redraw = false;
        event_handler_->OnMove(&redraw);

        if ( redraw )
            RePaint();
    }
}

void QGLWidgetEx::mousePressEvent(QMouseEvent *event)
{
    cursor_ = event->pos();
    DoMouseButton(event, true);
}

void QGLWidgetEx::mouseReleaseEvent(QMouseEvent *event)
{
    DoMouseButton(event, false);
}

void QGLWidgetEx::mouseDoubleClickEvent(QMouseEvent * event)
{
    if ( event_handler_ && (event->button() == Qt::LeftButton || event->button() == Qt::RightButton) )
    {
       bool redraw = false;
       event_handler_->OnDblClk(event->button() == Qt::LeftButton, &redraw);

       if ( redraw )
         RePaint();
    }
}

void QGLWidgetEx::wheelEvent(QWheelEvent *event)
{
    int zDelta = event->delta() / 120;

    if ( zDelta != 0 )
    {
        QPoint point = event->pos();

        point2 p = ToGlobal(point);

        ratio_ *= pow(1.1, zDelta);

        center_.x = p.x - (point.x() - width()  / 2.) * ratio_;
        center_.y = p.y + (point.y() - height() / 2.) * ratio_;

        ApplySize();
    }
}

void QGLWidgetEx::keyPressEvent(QKeyEvent * event)
{
    DoKey(event, true);
}

void QGLWidgetEx::keyReleaseEvent(QKeyEvent * event)
{
    DoKey(event, false);
}

void QGLWidgetEx::Advise ( IEvent * event )
{
    event_handler_ = event;
}

void QGLWidgetEx::Unadvise ()
{
    event_handler_ = 0;
}

void QGLWidgetEx::RePaint()
{
    //repaint(0, 0, width(), height());
    repaint();
}

void QGLWidgetEx::SetGlobalView( const point2 & center, double ratio )
{
    center_ = center;
    ratio_  = ratio;

    ApplySize();
}

void QGLWidgetEx::GetGlobalView( point2 * center, double * ratio )
{
    if ( center )
        *center = center_;
    if ( ratio )
        *ratio = ratio_;
}

QPoint QGLWidgetEx::GetCursorPos()
{
    return mapFromGlobal(QCursor::pos());
}

point2 QGLWidgetEx::GetCursorPosGlobal()
{
    return ToGlobal(GetCursorPos());
}

point2 QGLWidgetEx::ToGlobal( const QPoint & p )
{
    return point2(center_.x + (p.x() - width()  / 2.) * ratio_,
                  center_.y - (p.y() - height() / 2.) * ratio_);
}

QPoint QGLWidgetEx::ToScreen( const point2 & p )
{
    return QPoint(width()  / 2. + (p.x - center_.x) / ratio_,
                  height() / 2. - (p.y - center_.y) / ratio_);
}

void QGLWidgetEx::SetColor( double r, double g, double b, double a )
{
    glColor4d(lim(r), lim(g), lim(b), lim(a));
}

void QGLWidgetEx::SetLineWidth( double width )
{
   glLineWidth(float(width));
   glPointSize(float(width));
}

void QGLWidgetEx::Point( const point2 & pt )
{
   glBegin(GL_POINTS);
   glVertex2d(pt.x, pt.y);
   glEnd();
}

void QGLWidgetEx::Line( const point2 & pt0, const point2 & pt1 )
{
   glBegin(GL_LINES);
   glVertex2d(pt0.x, pt0.y);
   glVertex2d(pt1.x, pt1.y);
   glEnd();
}

void QGLWidgetEx::Circle( const point2 & center, double radius, bool fill, size_t num_subdivs )
{
   if ( num_subdivs == 0 )
      num_subdivs = std::max(5., fabs(radius / ratio_));

   glBegin(fill ? GL_POLYGON : GL_LINE_STRIP);
   for ( size_t i = 0; i <= num_subdivs; i++ )
   {
      double a = 2 * 3.14 * i / double(num_subdivs);
      double dx = radius * sin(a);
      double dy = radius * cos(a);
      glVertex2d(center.x + dx, center.y + dy);
   }
   glEnd();
}

void QGLWidgetEx::Triangle( const point2 & pt0, const point2 & pt1, const point2 & pt2, bool fill )
{
   glBegin(fill ? GL_TRIANGLES : GL_LINE_STRIP);
   glVertex2d(pt0.x, pt0.y);
   glVertex2d(pt1.x, pt1.y);
   glVertex2d(pt2.x, pt2.y);
   if ( !fill )
      glVertex2d(pt0.x, pt0.y);
   glEnd();
}

void QGLWidgetEx::Rectangle( const point2 & pt0, const point2 & pt1, bool fill )
{
   glBegin(fill ? GL_QUADS : GL_LINE_STRIP);
   glVertex2d(pt0.x, pt0.y);
   glVertex2d(pt0.x, pt1.y);
   glVertex2d(pt1.x, pt1.y);
   glVertex2d(pt1.x, pt0.y);
   if ( !fill )
      glVertex2d(pt0.x, pt0.y);
   glEnd();
}

void QGLWidgetEx::Points( const point2 * points, size_t n )
{
   glBegin(GL_POINTS);
   for ( size_t i = 0; i < n; i++ )
      glVertex2d(points[i].x, points[i].y);
   glEnd();
}

void QGLWidgetEx::Polyline( const point2 * points, size_t n )
{
   glBegin(GL_LINE_STRIP);
   for ( size_t i = 0; i < n; i++ )
      glVertex2d(points[i].x, points[i].y);
   glEnd();
}

int QGLWidgetEx::NewDisplayList()
{
   int id = glGenLists(1);

   glNewList(id, GL_COMPILE);

   return id;
}

void QGLWidgetEx::EndDisplayList()
{
   glEndList();
}

void QGLWidgetEx::CallDisplayList( int id )
{
   glCallList(id);
}

void QGLWidgetEx::DeleteDisplayList( int id )
{
   glDeleteLists(id, 1);
}

void QGLWidgetEx::PutText( const point2 & pt, const QString & text, int align )
{
   PutText(ToScreen(pt), text, align);
}

void QGLWidgetEx::PutText( const QPoint & pt, const QString & text, int align )
{
   if ( align == (AlignLeft|AlignBottom) )
      renderText(pt.x(), pt.y(), text);
   else
   {
      QSize sz = GetTextExtent(text);
      double dx = (align & AlignCenter) ? -sz.width() / 2 : (align & AlignRight) ? -sz.width() : 0;
      double dy = (align & AlignHCenter) ? sz.height() / 2 : (align & AlignTop) ? sz.height() : 0;
      renderText(pt.x() + dx, pt.y() + dy, text);
   }
}

QSize QGLWidgetEx::GetTextExtent( const QString & text )
{
   static QFont font;
   static QFontMetrics metrics(font);
   return QSize(metrics.width(text), metrics.height());
}

bool QGLWidgetEx::DoMouseButton( QMouseEvent *event, bool down )
{
    if ( event_handler_ )
    {
        bool redraw = false;
        if ( (event->button() == Qt::LeftButton || event->button() == Qt::RightButton)
             && event_handler_->OnButton(event->button() == Qt::LeftButton, down, &redraw) )
        {
            if ( redraw )
                RePaint();

            return true;
        }
    }

    return false;
}

bool QGLWidgetEx::DoKey( QKeyEvent *event, bool down )
{
    if ( event_handler_ )
    {
        bool redraw = false;
        if ( event_handler_->OnKey(event->key(), down, &redraw) )
        {
            if ( redraw )
                RePaint();

            return true;
        }
    }

    return false;
}

void QGLWidgetEx::ApplySize()
{
    glViewport(0, 0, width(), height());

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    point2 xy = ToGlobal(QPoint(0, 0));
    point2 XY = ToGlobal(QPoint(width(), height()));
    glOrtho(xy.x, XY.x, XY.y, xy.y, -1, 1);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    RePaint();
}

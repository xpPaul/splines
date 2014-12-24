#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#define NOMINMAX 1

#include "include/gui/glwidget/glwidget.h"
#include "include/splines/spline.h"
#include "include/splines/arclength.h"
#include "include/splines/localization.h"
#include "include/splines/builder.h"

struct scvt
{
    static bool eq( const point2 & a, const point2 & b ) { return norm(a - b) < 1e-5; }
};

typedef gsl::segment<double, point2, 3> spline_segment_type;
typedef gsl::spline_localization<gsl::spline_arclength<gsl::spline<spline_segment_type, scvt> > > spline_type;

namespace Ui {
class MainWindow;
}

class MainWindow
      : public QMainWindow
      , public QGLWidgetEx::IEvent
{
   Q_OBJECT

public:
   explicit MainWindow(QWidget *parent = 0);
   ~MainWindow();

public:
    void OnInit   ();
    void OnDraw   ();
    void OnMove   ( bool * redraw );
    bool OnButton ( bool left, bool down, bool * redraw );
    void OnDblClk ( bool left, bool * redraw );
    bool OnKey    ( int vkey, bool down, bool * redraw );

private slots:
    void on_actionSegments_triggered();
    void on_actionClosest_Point_triggered();
    void on_actionCurvature_triggered();
    void on_actionDerivatives_triggered();
    void on_actionFrenet_Frame_triggered();
    void on_actionArclength_triggered();

    void on_arclengthStep_editingFinished();

    void on_splineType_currentIndexChanged(const QString &arg1);

private:
    bool has_tangents() const;
    void build_spline();

private:
   Ui::MainWindow * ui_;
   QGLWidgetEx * gl_widget_;

   std::vector<point2> points_;
   std::vector<point2> tangents_; // for hermite
   spline_type spline_;

   size_t selected_point_;
   size_t selected_tangent_;
};

#endif // MAINWINDOW_H

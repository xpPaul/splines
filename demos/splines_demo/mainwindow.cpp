#include "mainwindow.h"
#include "ui_mainwindow.h"

static inline int sign( double x )
{
    return (x > 0) ? 1 : (x < 0) ? -1 : 0;
}

MainWindow::MainWindow(QWidget *parent)
   : QMainWindow(parent)
   , ui_(new Ui::MainWindow)
   , gl_widget_(new QGLWidgetEx())
{
   ui_->setupUi(this);
   gl_widget_->setParent(ui_->centralWidget);
   ui_->centralWidget->setLayout(new QHBoxLayout());
   ui_->centralWidget->layout()->addWidget(gl_widget_);
   gl_widget_->Advise(this);
}

MainWindow::~MainWindow()
{
   delete ui_;
}

void MainWindow::OnInit()
{
    gl_widget_->SetGlobalView(point2(0, 0), 3./gl_widget_->height());
}

void MainWindow::OnDraw()
{
    const point2 p = gl_widget_->GetCursorPosGlobal();

    double ratio;
    gl_widget_->GetGlobalView(0, &ratio);

    gl_widget_->SetColor(1, 1, 1);
    gl_widget_->SetLineWidth(5);
    if ( !points_.empty() )
        gl_widget_->Points(&points_[0], points_.size());

    const QString s = ui_->splineType->currentText();
    if ( has_tangents() )
    {
        gl_widget_->SetColor(1, 0, 0);
        gl_widget_->SetLineWidth(1);
        for ( size_t i = 0; i < std::min(points_.size(), tangents_.size()); i++ )
            gl_widget_->Line(points_[i], points_[i] + tangents_[i]);

        gl_widget_->SetLineWidth(5);
        for ( size_t i = 0; i < std::min(points_.size(), tangents_.size()); i++ )
            gl_widget_->Point(points_[i] + tangents_[i]);
    }

    gl_widget_->SetColor(1, 1, 1);
    gl_widget_->SetLineWidth(1.5);
    std::vector<point2> approx;
    spline_.approximate(ratio, back_inserter(approx));
    if ( !approx.empty() )
        gl_widget_->Polyline(&approx[0], approx.size());

    if ( ui_->actionSegments->isChecked() )
    {
        gl_widget_->SetLineWidth(5);

        std::vector<point2> segs;
        for ( size_t i = 1; i < spline_.size(); i++ )
            segs.push_back(spline_(i));

        if ( !segs.empty() )
            gl_widget_->Points(&segs[0], segs.size());
    }

    if ( spline_.size() > 0 )
    {
        double closest_point_t = 0;
        const double closest_point_d = spline_.distance(p, &closest_point_t);
        point2 closest_p = spline_(closest_point_t);

        if ( ui_->actionArclength->isChecked() )
        {
            const double marker_r = 5;

            gl_widget_->SetLineWidth(1);
            gl_widget_->SetColor(0, 1, 0);

            const double step = ui_->arclengthStep->value();
            const double len = spline_.length();
            for ( double s = step; s < len; s += step )
            {
                const double t = spline_.s2t(s);
                const point2 p = spline_(t);
                const point2 n = spline_.normal(t);

                gl_widget_->Line(p - marker_r * ratio * n, p + marker_r * ratio * n);
            }
        }

        if ( ui_->actionClosest_Point->isChecked() )
        {
            gl_widget_->SetLineWidth(1);
            gl_widget_->SetColor(1, 1, 0);
            gl_widget_->Line(p, closest_p);
            gl_widget_->PutText(p, QString("%1").sprintf("  d = %.2lf", closest_point_d));
            if ( ui_->actionArclength->isChecked() )
                gl_widget_->PutText(closest_p, QString("%1").sprintf("  t = %.2lf, s = %.2lf", closest_point_t, spline_.t2s(closest_point_t)));
            else
                gl_widget_->PutText(closest_p, QString("%1").sprintf("  t = %.2lf", closest_point_t));
        }

        if ( ui_->actionCurvature->isChecked() )
        {
            const double r = spline_.radius(closest_point_t);

            gl_widget_->SetLineWidth(1);
            gl_widget_->SetColor(0, 1, 1);

            if ( fabs(r) < 5 * std::max(gl_widget_->height(), gl_widget_->width()) * ratio )
            {
                const point2 c = closest_p + r * spline_.normal(closest_point_t);
                gl_widget_->Circle(c, r, false, 0);
                gl_widget_->Line(closest_p, c);
            }
            else
            {
                const double w = 1.5 * std::max(gl_widget_->height(), gl_widget_->width()) * ratio;
                const point2 t = spline_.direction(closest_point_t);

                gl_widget_->Line(closest_p, closest_p + sign(r) * w * spline_.normal(closest_point_t));
                gl_widget_->Line(closest_p - t * w, closest_p + t * w);
            }
        }

        if ( ui_->actionDerivatives->isChecked() )
        {
            gl_widget_->SetLineWidth(1);
            gl_widget_->SetColor(1, 0, 0);
            gl_widget_->Line(closest_p, closest_p + spline_.derivative<1>(closest_point_t));
            gl_widget_->SetColor(0, 1, 0);
            gl_widget_->Line(closest_p, closest_p + spline_.derivative<2>(closest_point_t));
            gl_widget_->SetColor(0, 0, 1);
            gl_widget_->Line(closest_p, closest_p + spline_.derivative<3>(closest_point_t));
        }

        if ( ui_->actionFrenet_Frame->isChecked() )
        {
            const double len = 100;

            gl_widget_->SetLineWidth(2);
            gl_widget_->SetColor(1, 0, 0);
            gl_widget_->Line(closest_p, closest_p + len * ratio * spline_.direction(closest_point_t));
            gl_widget_->SetColor(0, 1, 0);
            gl_widget_->Line(closest_p, closest_p + len * ratio * spline_.normal(closest_point_t));
        }
    }

    gl_widget_->SetLineWidth(1);
}

void MainWindow::OnMove( bool * redraw )
{
   *redraw = true;

    if ( !(QApplication::mouseButtons() & Qt::LeftButton) )
    {
        selected_point_ = -1;
        selected_tangent_ = -1;
    }

    if ( selected_point_ != size_t(-1) )
    {
        points_[selected_point_] = gl_widget_->GetCursorPosGlobal();
        build_spline();
    }
    if ( selected_tangent_ != size_t(-1) )
    {
        tangents_[selected_tangent_] = gl_widget_->GetCursorPosGlobal() - points_[selected_tangent_];
        build_spline();
    }
}

bool MainWindow::OnButton( bool left, bool down, bool * redraw )
{
    if ( !down )
        return false;

   const point2 p = gl_widget_->GetCursorPosGlobal();

   double ratio;
   gl_widget_->GetGlobalView(0, &ratio);

   if ( !left )
   {
       size_t point_to_erase = -1;
       double min_dist = 10;
       for ( size_t i = 0; i < points_.size(); i++ )
       {
           const double dist = distance(p, points_[i]) / ratio;
           if ( dist < min_dist )
           {
               point_to_erase = i;
               min_dist = dist;
           }
       }

       if ( point_to_erase != size_t(-1) )
       {
            points_.erase(points_.begin() + point_to_erase);
            if ( has_tangents() )
                tangents_.erase(tangents_.begin() + point_to_erase);
       }

       build_spline();
   }
   else
   {
       selected_point_ = -1;
       double min_dist = 10;
       for ( size_t i = 0; i < points_.size(); i++ )
       {
           const double dist = distance(p, points_[i]) / ratio;
           if ( dist < min_dist )
           {
               selected_point_ = i;
               min_dist = dist;
           }
       }
       if ( has_tangents() )
       {
           for ( size_t i = 0; i < tangents_.size(); i++ )
           {
               const double dist = distance(p, points_[i] + tangents_[i]) / ratio;
               if ( dist < min_dist )
               {
                   selected_point_ = -1;
                   selected_tangent_ = i;
                   min_dist = dist;
               }
           }
       }
   }

   *redraw = true;

   return true;
}

bool MainWindow::OnKey( int vkey, bool down, bool * redraw )
{
   if ( !down )
      return false;

   if ( vkey == Qt::Key_Delete )
   {
       points_.clear();
       tangents_.clear();
       build_spline();
   }

   *redraw = true;

   return true;
}

void MainWindow::OnDblClk( bool left, bool * redraw )
{
   if ( !left )
      return;

   const point2 p = gl_widget_->GetCursorPosGlobal();

   if ( has_tangents() )
   {
       const point2 p_prev = points_.empty() ? p : points_.back();

       if ( !tangents_.empty() )
       {
           const point2 p_prev = (points_.size() >= 2) ? (points_[points_.size() - 2]) : points_.back();
           tangents_.back() = (p - p_prev) / 2;
       }

       tangents_.push_back((p - p_prev) / 2);
   }

   points_.push_back(p);

   build_spline();

   *redraw = true;
}

bool MainWindow::has_tangents() const
{
    const QString s = ui_->splineType->currentText();
    return s == "Hermite";
}

void MainWindow::build_spline()
{
    const QString s = ui_->splineType->currentText();

    if ( s == "B-Spline" )
        spline_ = gsl::spline_builder<spline_type, gsl::b_spline>(points_.begin(), points_.end());
    else if ( s == "Catmull-Rom" )
        spline_ = gsl::spline_builder<spline_type, gsl::catmull_rom_spline>(points_.begin(), points_.end());
    else if ( s == "Bezier" )
    {
        if ( points_.size() > 4 )
            points_.resize(4);

        spline_ = gsl::spline_builder<spline_type, gsl::bezier_spline>(points_.begin(), points_.end());
    }
    else if ( s == "Hermite" )
    {
        if ( tangents_.size() != points_.size() )
        {
            tangents_.resize(points_.size());
            for ( size_t i = 0; i < points_.size(); i++ )
            {
                const point2 p_prev = (i > 0) ? points_[i - 1] : points_.front();
                const point2 p_next = (i + 1 < points_.size()) ? points_[i + 1] : points_.back();

                tangents_[i] = (p_next - p_prev) / 2;
            }
        }

        std::vector<spline_segment_type> segs;
        for ( size_t i = 0; i + 1 < points_.size(); i++ )
            segs.push_back(gsl::hermite_segment<spline_segment_type>(points_[i], points_[i + 1], tangents_[i], tangents_[i + 1]));
        spline_.assign(segs.begin(), segs.end());
    }

    spline_.set_localization_accuracy(0.01);
    spline_.set_parametrization_accuracy(0.01);

    // update info
    ui_->numPoints->setText(QString::number(points_.size()));
    ui_->totalLength->setText(QString::number(spline_.length(), 'f', 2));
}

void MainWindow::on_actionSegments_triggered()
{
    gl_widget_->RePaint();
}

void MainWindow::on_actionClosest_Point_triggered()
{
    gl_widget_->RePaint();
}

void MainWindow::on_actionCurvature_triggered()
{
    gl_widget_->RePaint();
}

void MainWindow::on_actionDerivatives_triggered()
{
    gl_widget_->RePaint();
    ui_->actionFrenet_Frame->setChecked(false);
}

void MainWindow::on_actionFrenet_Frame_triggered()
{
    gl_widget_->RePaint();
    ui_->actionDerivatives->setChecked(false);
}

void MainWindow::on_actionArclength_triggered()
{
    gl_widget_->RePaint();
}

void MainWindow::on_arclengthStep_editingFinished()
{
    gl_widget_->RePaint();
}

void MainWindow::on_splineType_currentIndexChanged(const QString &)
{
    build_spline();
    gl_widget_->RePaint();
}

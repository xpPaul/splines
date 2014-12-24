splines
=======

Generic splines library, almost arbitrary user point type can be interpolated.

Several types of spline presented (B-Spline, Hermite spline, Catmull-Rom spline, Bezier spline) and other spline types can be added by user.

Spline support following operations:
  - interpolation
  - derivative calculation
  - curvature/radius/torsion calculation
  - approximation
  - arclength parametrization
  - closest point
  - frenet frame

See include\splines\user.txt for more details.

See demos\splines_demo for usage sample

![bspline with arclength and closest point](https://raw.github.com/xpPaul/splines/master/splines_shots/bspline_arclength_closestpoint.png)
![bspline with derivatives](https://raw.github.com/xpPaul/splines/master/splines_shots/bspline_derivatives.png)
![catmullrom with curvature](https://raw.github.com/xpPaul/splines/master/splines_shots/catmullrom_cuvature.png)
![hermite with frenetframe](https://raw.github.com/xpPaul/splines/master/splines_shots/hermite_frenetframe.png)

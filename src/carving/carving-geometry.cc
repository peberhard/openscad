/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2014 Pascal Eberhard <pascal.eberhard@38plumes.fr>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "calc.h"
#include "cgalutils.h"
#include "mathc99.h"
#include "CGAL_Nef3_workaround.h"
#include "carving/carving-geometry.h"
#include <math.h>
#include <assert.h>
#include <errno.h>
#include <limits>
#include <boost/math/special_functions/fpclassify.hpp>

typedef CGAL_Polyhedron::Vertex_iterator        Vertex_iterator;
typedef CGAL_Polyhedron::Halfedge_iterator      Halfedge_iterator;
typedef CGAL_Polyhedron::Point_iterator         Point_iterator;

using boost::math::isfinite;

#define DRILL_MAX_FN 12
#define EPSILON 0.01    // required for overlap for propoer substraction

struct point2d
{
  double x, y;
};


/*
 * Geometry used to start or end a mill with an half circle of the size of the end-mill tool.
 */
int generate_half_cylinder(PolySet *p, double x, double y, double z, double abs_arc_x1, double abs_arc_y1,
    double abs_arc_x2, double abs_arc_y2, bool ccw, double cylinder_h, double fn, double fs, double fa)
{
  if (ccw) {
    double tmpx = abs_arc_x1;
    abs_arc_x1 = abs_arc_x2;
    abs_arc_x2 = tmpx;
    double tmpy = abs_arc_y1;
    abs_arc_y1 = abs_arc_y2;
    abs_arc_y2 = tmpy;
  }
  double arc_x1 = abs_arc_x1 - x;
  double arc_y1 = abs_arc_y1 - y;
  double arc_x2 = abs_arc_x2 - x;
  double arc_y2 = abs_arc_y2 - y;
  double r = sqrt(arc_x1 * arc_x1 + arc_y1 * arc_y1);
  PRINTDB("generate_half_cylinder x=%g, y=%g, z=%g, arc_x1=%g, arc_y1=%g, arc_x2=%g, arc_y2=%g, cylinder_h=%g, fn=%g, fs=%g, fa=%g, r=%g",
          x % y % z % arc_x1 % arc_y1 % arc_x2 % arc_y2 % cylinder_h % fn % fs % fa % r);
  if (!(isfinite(x) && isfinite(y) && isfinite(z) && isfinite(arc_x1) && isfinite(arc_y1) && isfinite(arc_x2)
      && isfinite(arc_y2) && isfinite(cylinder_h) && isfinite(fn) && isfinite(fs) && isfinite(fa) && cylinder_h > 0)) {
    PRINTB("WARNING: Carving: generate_half_cylinder invalid parameter(s) x=%g, y=%g, z=%g, arc_x1=%g, arc_y1=%g, arc_x2=%g, arc_y2=%g, cylinder_h=%g, fn=%g, fs=%g, fa=%g, r=%g",
            x % y % z % arc_x1 % arc_y1 % arc_x2 % arc_y2 % cylinder_h % fn % fs % fa % r);
    return -EINVAL;
  }

  double angle1 = atan2(arc_y1, arc_x1);
  double angle2 = atan2(arc_y2, arc_x2);
  double angle = angle2 - angle1;
  if (angle >= 0) {
    angle -= 2 * M_PI;
  }
  int fragments = Calc::get_fragments_from_r(r, fn, fs, fa);
  if (fragments > DRILL_MAX_FN/2)
    fragments = DRILL_MAX_FN/2;

  point2d *circle = new point2d[fragments + 1];
  for (int i = 0; i < fragments; i++) {
    double phi = angle1 + (angle * i) / fragments;
    circle[i].x = r * cos(phi) + x;
    circle[i].y = r * sin(phi) + y;
  }
  fragments++;
  circle[fragments - 1].x = r * cos(angle2) + x;
  circle[fragments - 1].y = r * sin(angle2) + y;

  double z1 = z;
  double z2 = z1 + cylinder_h;

  // generate circle
  for (int i = 0; i < fragments - 1; i++) {
    // side
    p->append_poly();
    p->insert_vertex(circle[i + 1].x, circle[i + 1].y, z1);
    p->insert_vertex(circle[i + 1].x, circle[i + 1].y, z2);
    p->insert_vertex(circle[i].x, circle[i].y, z2);
    p->insert_vertex(circle[i].x, circle[i].y, z1);
  }
  // bottom
  p->append_poly();
  for (int i = fragments - 1; i >= 0; i--)
    p->insert_vertex(circle[i].x, circle[i].y, z1);
  // top
  p->append_poly();
  for (int i = 0; i < fragments; i++)
    p->insert_vertex(circle[i].x, circle[i].y, z2);

  delete []circle;
  return 0;
}


int generate_rectangle(PolySet *p, double x, double y, double z, double length, double width, double height)
{
  double x1 = x;
  double y1 = y;
  double x2 = x + length;
  double y2 = y + width;
  double z1 = z;
  double z2 = z + height;
  PRINTDB("generate_rectangle [%g,%g,%g] [%g,%g,%g]", x1 % y1 % z1 % x2 % y2 % z2);
  if (!(isfinite(x) && isfinite(y) && isfinite(z) && isfinite(length) && isfinite(width) && isfinite(height)
      && height > 0)) {
    PRINTB("WARNING: Carving: generate_rectangle invalid parameter(s) x=%g, y=%g, z=%g, length=%g, width=%g, height=%g",
            x % y % z % length % width % height);
    return -EINVAL;
  }

  p->append_poly(); // bottom
  p->append_vertex(x1, y1, z1);
  p->append_vertex(x1, y2, z1);
  p->append_vertex(x2, y2, z1);
  p->append_vertex(x2, y1, z1);

  p->append_poly(); // top
  p->append_vertex(x1, y1, z2);
  p->append_vertex(x2, y1, z2);
  p->append_vertex(x2, y2, z2);
  p->append_vertex(x1, y2, z2);

  p->append_poly(); // side1
  p->append_vertex(x1, y1, z1);
  p->append_vertex(x2, y1, z1);
  p->append_vertex(x2, y1, z2);
  p->append_vertex(x1, y1, z2);

  p->append_poly(); // side 2
  p->append_vertex(x2, y1, z1);
  p->append_vertex(x2, y2, z1);
  p->append_vertex(x2, y2, z2);
  p->append_vertex(x2, y1, z2);

  p->append_poly(); // slide 3
  p->append_vertex(x1, y2, z1);
  p->append_vertex(x1, y2, z2);
  p->append_vertex(x2, y2, z2);
  p->append_vertex(x2, y2, z1);

  p->append_poly(); // side4
  p->append_vertex(x1, y1, z1);
  p->append_vertex(x1, y1, z2);
  p->append_vertex(x1, y2, z2);
  p->append_vertex(x1, y2, z1);

#ifdef ENABLE_CARVING_UNSTABLE
  CGAL_Polyhedron polyhedron;
  bool err = CGALUtils::createPolyhedronFromPolySet(*p, polyhedron);
  if (err) {
    PRINTD("generate_rectangle createPolyhedronFromPolySet failed");
    assert(0);
  }
  if (! polyhedron.is_closed()) {
    PRINTD("generate_rectangle polyhedron.is_closed false");
    for (CGAL_Polyhedron::Halfedge_const_iterator i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end();
        ++i) {
      if (i->is_border()) {
        PRINTD("generate_rectangle halfedge is_border false");
      }
    }
    assert(0);
  }
  if (! polyhedron.is_valid(false, 0)) {
    polyhedron.is_valid(true, 0);
    PRINTD("generate_rectangle polyhedron.is_valid false");
    assert(0);
  }
#endif

  return 0;
}


int generate_rectangle(PolySet *p, double x1, double y1, double z1, double x2, double y2, double z2, double diam,
    double fn, double fs, double fa, bool rounded)
{
  double dx = x2 - x1;
  double dy = y2 - y1;
  //double hypo = sqrt(dx*dx + dy*dy);
  double alpha = atan2(dy, dx);
  double x11 = x1 - diam * sin(alpha);
  double y11 = y1 + diam * cos(alpha);
  double x12 = x1 + diam * sin(alpha);
  double y12 = y1 - diam * cos(alpha);
  double x21 = x2 - diam * sin(alpha);
  double y21 = y2 + diam * cos(alpha);
  double x22 = x2 + diam * sin(alpha);
  double y22 = y2 - diam * cos(alpha);
  z1 -= EPSILON;    // required for proper geometric subtraction when step_down < thickness
  if (z2 > 0) {
    z2 = 0;
  }
  PRINTDB("generate_rectangle alpha %g, sin %g, cos %g, [%g, %g, %g] [%g, %g, %g] x11 %g, y11 %g, rounded %d, epsilon %g",
          alpha % sin(alpha) % cos(alpha) % x1 % y1 % z1 % x2 % y2 % z2 % x11 % y11 % rounded % EPSILON);
  if (!(isfinite(x1) && isfinite(y1) && isfinite(z1) && isfinite(x2) && isfinite(y2) && isfinite(z2) &&
      isfinite(diam) && (z2 - z1) > 0)) {
    PRINTB("WARNING: Carving: generate_rectangle [%g, %g, %g] [%g, %g, %g] diam %g, rounded %d, height (z2 - z1) %g",
           x1 % y1 % z1 % x2 % y2 % z2 % diam % rounded % (z2 - z1));
    return -EINVAL;
  }
  if (rounded) {
    generate_half_cylinder(p, x1, y1, z1, x12, y12, x11, y11, false, z2 - z1, fn, fs, fa); // side 1
    generate_half_cylinder(p, x2, y2, z1, x21, y21, x22, y22, false, z2 - z1, fn, fs, fa); // side 3
  } else {
    p->append_poly(); // side 1
    p->append_vertex(x11, y11, z1);
    p->append_vertex(x12, y12, z1);
    p->append_vertex(x12, y12, z2);
    p->append_vertex(x11, y11, z2);

    p->append_poly(); // side3
    p->append_vertex(x22, y22, z1);
    p->append_vertex(x21, y21, z1);
    p->append_vertex(x21, y21, z2);
    p->append_vertex(x22, y22, z2);
  }

  p->append_poly(); // top
  p->append_vertex(x11, y11, z2);
  p->append_vertex(x12, y12, z2);
  p->append_vertex(x22, y22, z2);
  p->append_vertex(x21, y21, z2);

  p->append_poly(); // side2
  p->append_vertex(x22, y22, z1);
  p->append_vertex(x22, y22, z2);
  p->append_vertex(x12, y12, z2);
  p->append_vertex(x12, y12, z1);

  p->append_poly(); // side4
  p->append_vertex(x11, y11, z1);
  p->append_vertex(x11, y11, z2);
  p->append_vertex(x21, y21, z2);
  p->append_vertex(x21, y21, z1);

  p->append_poly(); // bottom
  p->append_vertex(x11, y11, z1);
  p->append_vertex(x21, y21, z1);
  p->append_vertex(x22, y22, z1);
  p->append_vertex(x12, y12, z1);

#ifdef ENABLE_CARVING_UNSTABLE
  CGAL_Polyhedron polyhedron;
  bool err = CGALUtils::createPolyhedronFromPolySet(*p, polyhedron);
  if (err) {
    PRINTD("generate_rectangle createPolyhedronFromPolySet failed");
    assert(0);
  }
  if (! polyhedron.is_closed()) {
    PRINTD("generate_rectangle polyhedron.is_closed false");
    for (CGAL_Polyhedron::Halfedge_const_iterator i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end();
        ++i) {
      if (i->is_border()) {
        PRINTD("generate_rectangle halfedge is_border false");
      }
    }
    assert(0);
  }
  if (! polyhedron.is_valid(false, 0)) {
    polyhedron.is_valid(true, 0);
    PRINTD("generate_rectangle polyhedron.is_valid false");
    assert(0);
  }
#endif

  return 0;
}


/*
 * Make the cylinder related to a drill hole
 */
int generate_cylinder(PolySet *p, double h, double r, double x, double y, double z, double fn, double fs, double fa)
{
  double z1 = z;
  double z2 = z1 + h;
  if (z2 > 0) {
    z2 = 0;
  }

  PRINTDB("generate_cylinder h=%g, r=%g, x=%g, y=%g, z=%g, fn=%g, fs=%g, fa=%g", h % r % x % y % z % fn % fs % fa);
  if (!(isfinite(h) && isfinite(r) && isfinite(x) && isfinite(y) && isfinite(z) && isfinite(fn) && isfinite(fs)
      && isfinite(fa) && r > 0 && h > 0)) {
    PRINTB("WARNING: Carving: generate_cylinder invalid argument(s), h=%g, r=%g, x=%g, y=%g, z=%g, fn=%g, fs=%g, fa=%g",
            h % r % x % y % z % fn % fs % fa);
    return -EINVAL;
  }

  // generate circle
  int fragments = Calc::get_fragments_from_r(r, fn, fs, fa);
  if (fragments > DRILL_MAX_FN)
    fragments = DRILL_MAX_FN;

  point2d *circle = new point2d[fragments];
  for (int i = 0; i < fragments; i++) {
    double phi = (M_PI * 2 * i) / fragments;
    circle[i].x = r * cos(phi) + x;
    circle[i].y = r * sin(phi) + y;
  }

  // generate cylinder fragmented sides
  for (int i = 0; i < fragments; i++) {
    int j = (i + 1) % fragments;
    p->append_poly();
    p->insert_vertex(circle[i].x, circle[i].y, z1);
    p->insert_vertex(circle[i].x, circle[i].y, z2);
    p->insert_vertex(circle[j].x, circle[j].y, z2);
    p->insert_vertex(circle[j].x, circle[j].y, z1);
    //PRINTDB("generate_cylinder ix=%g, iy=%g, z1=%g, jx=%g, jy=%g, z2=%g",
    //    circle[i].x % circle[i].y % z1 % circle[j].x % circle[j].y % z2);
  }
  // bottom
  p->append_poly();
  for (int i = 0; i < fragments; i++) {
    p->insert_vertex(circle[i].x, circle[i].y, z1);
    //PRINTDB("generate_cylinder x=%g, y=%g, z=%g", circle[i].x % circle[i].y % z1);
  }
  // top
  p->append_poly();
  for (int i = 0; i < fragments; i++) {
    p->insert_vertex(circle[fragments - 1 - i].x, circle[fragments - 1 - i].y, z2);
    //PRINTDB("generate_cylinder x=%g, y=%g, z=%g", circle[fragments-1-i].x % circle[fragments-1-i].y % z2);
  }
  delete[] circle;

#ifdef ENABLE_CARVING_UNSTABLE
  CGAL_Polyhedron polyhedron;
  bool err = CGALUtils::createPolyhedronFromPolySet(*p, polyhedron);
  if (err) {
    PRINTD("generate_cylinder createPolyhedronFromPolySet failed");
    assert(0);
  }
  if (! polyhedron.is_closed()) {
    PRINTD("generate_cylinder polyhedron.is_closed false");
    for (CGAL_Polyhedron::Halfedge_const_iterator i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end();
        ++i) {
      if (i->is_border()) {
        PRINTD("generate_cylinder halfedge is_border false");
      }
    }
    assert(0);
  }
  if (! polyhedron.is_valid(false, 0)) {
    polyhedron.is_valid(true, 0);
    PRINTD("generate_cylinder polyhedron.is_valid false");
    assert(0);
  }
#endif

  return 0;
}


int generate_arc(PolySet *p, double x, double y, double z, double abs_arc_x1, double abs_arc_y1, double abs_arc_x2,
    double abs_arc_y2, bool ccw, double cylinder_h, double cylinder_r, double fn, double fs, double fa)
{
  if (ccw) {
    double tmpx = abs_arc_x1;
    abs_arc_x1 = abs_arc_x2;
    abs_arc_x2 = tmpx;
    double tmpy = abs_arc_y1;
    abs_arc_y1 = abs_arc_y2;
    abs_arc_y2 = tmpy;
  }
  int rc;
  bool circle = abs_arc_x1 == abs_arc_x2 && abs_arc_y1 == abs_arc_y2;
  double arc_x1 = abs_arc_x1 - x;
  double arc_y1 = abs_arc_y1 - y;
  double arc_x2 = abs_arc_x2 - x;
  double arc_y2 = abs_arc_y2 - y;
  double r1 = sqrt(arc_x1 * arc_x1 + arc_y1 * arc_y1) - cylinder_r;
  double r2 = r1 + 2 * cylinder_r;
  z -= EPSILON;    // required for proper geometric subtraction when step_down < thickness
  cylinder_h += EPSILON;
  if (z + cylinder_h > 0) {
    cylinder_h = - z;
  }

  if (circle && r1 <= 0) { // very small circle where center of the path of the tool < tool radius
    return generate_cylinder(p, cylinder_h, r2, x, y, z, fn, fs, fa);
  }

  PRINTDB("generate_arc x=%g, y=%g, z=%g, arc_x1=%g, arc_y1=%g, arc_x2=%g, arc_y2=%g, cylinder_h=%g, cylinder_r=%g, fn=%g, fs=%g, fa=%g, r1 %g, r2 %g",
          x % y % z % arc_x1 % arc_y1 % arc_x2 % arc_y2 % cylinder_h % cylinder_r % fn % fs % fa % r1 % r2);
  if (!(isfinite(x) && isfinite(y) && isfinite(z) && isfinite(arc_x1) && isfinite(arc_y1) && isfinite(arc_x2)
      && isfinite(arc_y2) && isfinite(cylinder_h) && isfinite(cylinder_r) && isfinite(fn) && isfinite(fs)
      && isfinite(fa) && cylinder_h > 0 && cylinder_r > 0)) {
    PRINTB("WARNING: Carving: generate_arc invald arg x=%g, y=%g, z=%g, arc_x1=%g, arc_y1=%g, arc_x2=%g, arc_y2=%g, cylinder_h=%g, cylinder_r=%g, fn=%g, fs=%g, fa=%g, (r1=%g r2=%g)",
            x % y % z % arc_x1 % arc_y1 % arc_x2 % arc_y2 % cylinder_h % cylinder_r % fn % fs % fa % r1 % r2);
    return -EINVAL;
  }

  double angle1 = atan2(arc_y1, arc_x1);
  double angle2 = atan2(arc_y2, arc_x2);
  double angle = angle2 - angle1;
  if (angle >= 0) {
    angle -= 2 * M_PI;
  }
  int fragments = Calc::get_fragments_from_r(r1, fn, fs, fa);
  //PRINTDB("generate_arc a1=%g, a2=%g, a=%g fragments=%d", angle1 % angle2 % angle % fragments);

  double z1 = z;
  double z2 = z1 + cylinder_h;
  double x00;
  double y00;
  double x01;
  double y01;
  double x10;
  double y10;
  double x11;
  double y11;

  // generate circle
  for (int ii = 0; ii < fragments; ii++) {
    double phi0 = angle1 + (angle * ii) / fragments;
    double phi1 = angle1 + (angle * (ii + 1)) / fragments;
    // side inside
    x00 = r1 * cos(phi0) + x;
    y00 = r1 * sin(phi0) + y;
    x01 = r1 * cos(phi1) + x;
    y01 = r1 * sin(phi1) + y;

    x10 = r2 * cos(phi0) + x;
    y10 = r2 * sin(phi0) + y;
    x11 = r2 * cos(phi1) + x;
    y11 = r2 * sin(phi1) + y;

    if (ii == 0 && !circle) {
      /*
       p->append_poly();
       p->insert_vertex(x00, y00, z1);
       p->insert_vertex(x10, y10, z1);
       p->insert_vertex(x10, y10, z2);
       p->insert_vertex(x00, y00, z2);
       //*/
      rc = generate_half_cylinder(p, abs_arc_x1, abs_arc_y1, z, x00, y00, x10, y10, false, cylinder_h, fn, fs, fa);
      if (rc < 0) {
        return rc;
      }
    }
//*
    // side inside
    p->append_poly();
    p->insert_vertex(x00, y00, z1);
    p->insert_vertex(x00, y00, z2);
    p->insert_vertex(x01, y01, z2);
    p->insert_vertex(x01, y01, z1);
    // side outside
    p->append_poly();
    p->insert_vertex(x11, y11, z1);
    p->insert_vertex(x11, y11, z2);
    p->insert_vertex(x10, y10, z2);
    p->insert_vertex(x10, y10, z1);
    // bottom
    p->append_poly();
    p->insert_vertex(x01, y01, z1);
    p->insert_vertex(x11, y11, z1);
    p->insert_vertex(x10, y10, z1);
    p->insert_vertex(x00, y00, z1);
    // top
    p->append_poly();
    p->insert_vertex(x00, y00, z2);
    p->insert_vertex(x10, y10, z2);
    p->insert_vertex(x11, y11, z2);
    p->insert_vertex(x01, y01, z2);
//*/
  }
  if (!circle) {
    /*
     p->append_poly();
     p->insert_vertex(x01, y01, z1);
     p->insert_vertex(x01, y01, z2);
     p->insert_vertex(x11, y11, z2);
     p->insert_vertex(x11, y11, z1);
     //*/
    //*
    rc = generate_half_cylinder(p, abs_arc_x2, abs_arc_y2, z, x11, y11, x01, y01, false, cylinder_h, fn, fs, fa);
    if (rc < 0) {
      return rc;
    }
    //*/
  }

#ifdef ENABLE_CARVING_UNSTABLE
  CGAL_Polyhedron polyhedron;
  bool err = CGALUtils::createPolyhedronFromPolySet(*p, polyhedron);
  if (err) {
    PRINTD("generate_arc createPolyhedronFromPolySet failed");
    assert(0);
  }
  if (! polyhedron.is_closed()) {
    PRINTD("generate_arc polyhedron.is_closed false");
    for (CGAL_Polyhedron::Halfedge_const_iterator i = polyhedron.halfedges_begin(); i != polyhedron.halfedges_end();
        ++i) {
      if (i->is_border()) {
        PRINTD("generate_arc halfedge is_border false");
      }
    }
    //assert(0);
    PRINTB("WARNING: Carving: generate_arc not closed, x=%g, y=%g, z=%g, arc_x1=%g, arc_y1=%g, arc_x2=%g, arc_y2=%g, cylinder_h=%g, cylinder_r=%g, fn=%g, fs=%g, fa=%g, r1 %g, r2 %g",
        x % y % z % arc_x1 % arc_y1 % arc_x2 % arc_y2 % cylinder_h % cylinder_r % fn % fs % fa % r1 % r2);
  }
  if (! polyhedron.is_valid(false, 0)) {
    polyhedron.is_valid(true, 0);
    PRINTD("generate_arc polyhedron.is_valid false");
    assert(0);
  }
#endif

  return 0;
}


static std::string halfedge_key(const Halfedge_iterator &lhs)
{
  std::stringstream s;
  CGAL_Point_3 pt = lhs->vertex()->point();
  s << "[" << pt.x().to_double() << ", " << pt.y().to_double() << ", " << pt.z().to_double() << "] -> ";
  pt = lhs->next()->vertex()->point();
  s  << "[" << pt.x().to_double() << ", " << pt.y().to_double() << ", " << pt.z().to_double() << "]";
  return s.str();
}


static void nef_polyhedron_bottom_left(const CGAL_Point_3 &pt, double &x, double &y, double &z)
{
  if (pt.x().to_double() < x)
    x = pt.x().to_double();
  if (pt.y().to_double() < y)
    y = pt.y().to_double();
  if (pt.z().to_double() < z)
    z = pt.z().to_double();
}


static bool nef_polyhedron_is_closer(double x, double y, double z, const CGAL_Point_3 &pt, double &last_dist)
{
  double dx =  x - pt.x().to_double();
  double dy =  y - pt.y().to_double();
  double dz =  z - pt.z().to_double();
  double new_dist = dx*dx + dy*dy + dz*dz;
  if (new_dist < last_dist) {
      last_dist = new_dist;
      return true;
  }
  return false;
}


static Halfedge_iterator nef_polyhedron_get_closest_point(CGAL_Polyhedron &ph, double x, double y, double z)
{
  Halfedge_iterator found_iter = ph.halfedges_end();
  double last_dist = std::numeric_limits<double>::max();
  for(Halfedge_iterator he_iter = ph.halfedges_begin(); he_iter != ph.halfedges_end(); ++he_iter) {
    CGAL_Point_3 pt = he_iter->vertex()->point();
    if (nef_polyhedron_is_closer(x, y, z, pt, last_dist))
        found_iter = he_iter;
  }
  return found_iter;
}


int extract_volume_from_net_polyhedron(const CGAL_Nef_polyhedron &nef, double x, double y, double z,
    shared_ptr<PolySet> ps)
{
  CGAL_Polyhedron ph;
  bool err = nefworkaround::convert_to_Polyhedron<CGAL_Kernel3>(*(nef.p3), ph);
  if (err) {
    PRINTB("ERROR: extract_volume_from_polyhedron err %d", err);
  }
  std::set<std::string> already_saw;
  std::list<Halfedge_iterator> queue;
  Halfedge_iterator he_iter = nef_polyhedron_get_closest_point(ph, x, y, z);
  PRINTDB("extract_volume_from_net_polyhedron [%g, %g, %g] closest is [%g, %g, %g]",
      x % y % z % he_iter->vertex()->point().x().to_double() % he_iter->vertex()->point().y().to_double() %
      he_iter->vertex()->point().z().to_double());
  assert(he_iter != ph.halfedges_end());
  queue.push_back(he_iter);
  already_saw.insert(halfedge_key(he_iter));

  double d0x = std::numeric_limits<double>::max();
  double d0y = std::numeric_limits<double>::max();
  double d0z = std::numeric_limits<double>::max();

  while (!queue.empty()) {
    Halfedge_iterator begin = queue.front();
    Halfedge_iterator it = begin;
    queue.pop_front();
    ps->append_poly();
    std::cout << "halfedge face ";
    do {
      std::string k = halfedge_key(it);
      if (already_saw.count(k) == 0) {
//        printf("                  adding halfedge %s\n", k.c_str());
        queue.push_back(it->opposite());
        already_saw.insert(k);
      }
      CGAL_Point_3 pt = it->vertex()->point();
      nef_polyhedron_bottom_left(pt, d0x, d0y, d0z);
      PRINTDB("pt [%g, %g, %g]", pt.x().to_double() % pt.y().to_double() % pt.z().to_double());
      ps->insert_vertex(pt.x().to_double(), pt.y().to_double(), pt.z().to_double());

      it = it->prev();
    } while (it->vertex()->point() != begin->vertex()->point());

    std::cout << std::endl;

  }
  // Translate object to [0, 0 ,0]
  PRINTDB("Translate volume by [%g, %g, %g]", d0x % d0y % d0z);
  Transform3d mat = Eigen::Translation3d(-d0x, -d0y, -d0z) * Eigen::Scaling(1.,1.,1.);
  ps->transform(mat);

  return 0;
}


const Transform2d transform3d_to_transform2d(const Transform3d &matrix3d)
{
  Transform2d matrix2d;
  matrix2d.matrix() << matrix3d(0, 0), matrix3d(0, 1), matrix3d(0, 3),
                       matrix3d(1, 0), matrix3d(1, 1), matrix3d(1, 3),
                       matrix3d(3, 0), matrix3d(3, 1), matrix3d(3, 3);
  return matrix2d;
}


double arc_angle(double center_x, double center_y, double arc_x1, double arc_y1, double arc_x2, double arc_y2)
{
  // computing the clockwise angle of the arc
  // dot = x1*x2 + y1*y2      # dot product
  // det = x1*y2 - y1*x2      # determinant
  // angle = atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
  double x1 = arc_x1 - center_x;
  double y1 = arc_y1 - center_y;
  double x2 = arc_x2 - center_x;
  double y2 = arc_y2 - center_y;
  double dot = x1 * x2 + y1 * y2;
  double det = x1 * y2 - y1 * x2;
  double angle = atan2(det, dot) * 180 / M_PI;
  return angle;
}

bool is_matrix_reversing_angle(const Transform2d &matrix2d)
{
  Eigen::Vector2d center(0,0);
  Eigen::Vector2d arc1(0,1);
  Eigen::Vector2d arc2(1,0);
  double angle0 = arc_angle(center.x(), center.y(), arc1.x(), arc1.y(), arc2.x(), arc2.y());
  center = matrix2d * center;
  arc1 = matrix2d * arc1;
  arc2 = matrix2d * arc2;
  double angle1 = arc_angle(center.x(), center.y(), arc1.x(), arc1.y(), arc2.x(), arc2.y());

  int sign = (angle0 < 0 ? -1 : 1) * (angle1 < 0 ? -1 : 1);

  return sign < 0;
}


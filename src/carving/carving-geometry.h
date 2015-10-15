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

#pragma once

#include "polyset.h"
#include "linalg.h"

#define DEFAULT_Z 0

/* Geometry helper functions */

int generate_tool(PolySet *p, double x1, double y1, double z1, double x2, double y2, double z2, double diam, double dz);

int generate_rectangle(PolySet *p, double x, double y, double z, double length, double width, double height);

int generate_rectangle(PolySet *p, double x1, double y1, double z1, double x2, double y2, double z2, double diam,
    double fn, double fs, double fa, bool rounded = true);

int generate_cylinder(PolySet *p, double h, double r, double x, double y, double z, double fn, double fs, double fa);

int generate_arc(PolySet *p, double x, double y, double z, double arc_x1, double arc_y1, double arc_x2, double arc_y2,
    bool ccw, double cylinder_h, double cylinder_r, double fn, double fs, double fa);

int extract_volume_from_net_polyhedron(const class CGAL_Nef_polyhedron &nef, double x, double y, double z,
    shared_ptr<PolySet> ps);

// Create 2d transform matrix from 3d transform matrix
const Transform2d transform3d_to_transform2d(const Transform3d &matrix3d);

/* compute the angle of an arc in a circle
 * return the angle
 */
double arc_angle(double center_x, double center_y, double arc_x1, double arc_y1, double arc_x2, double arc_y2);

bool is_matrix_reversing_angle(const Transform2d &matrix2d);

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

#include <sstream>
#include <list>
#include <boost/shared_ptr.hpp>
#include "linalg.h"

using boost::shared_ptr;


enum carving_op_type_e
{
  CARVING_OP_PATH2D,			// handle milling start and end.
  CARVING_OP_RAPID_MOVE_XY,	// G0 http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G0
  CARVING_OP_RAPID_MOVE_Z,		// G0
  CARVING_OP_LINEAR_MOVE_XY,	// G1 http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G1
  CARVING_OP_LINEAR_MOVE_Z,	// G1
  CARVING_OP_ARC_MOVE_XY,		// G2 / G3
};

class CarvingOperation
{
public:
  static CarvingOperation *newGCodeOpPath2d(double x, double y, double z);
  static CarvingOperation *newGCodeOpRapidMoveXY(double x, double y);
  static CarvingOperation *newGCodeOpRapidMoveZ(double z);
  static CarvingOperation *newGCodeOpLinearMoveXY(double x, double y);
  static CarvingOperation *newGCodeOpLinearMoveZ(double z);
  static CarvingOperation *newGCodeOpArcMoveXY(double x, double y, double i, double j, double p, bool ccw);

  /* instance methods */

  std::string toString() const;
  std::string name() const;
  int getType() const;
  double getX() const;
  double getY() const;
  double getZ() const;
  double getI() const;
  double getJ() const;
  double getP() const;
  bool getCcw() const;
  void reverseCcw() const;
  CarvingOperation *reverseTo(double to_x, double to_y) const;
  void transform(const Transform2d &matrix2d) const;

  std::list<shared_ptr<CarvingOperation> > children;; // TODO Carving: private?

private:
  CarvingOperation(carving_op_type_e type, double x, double y, double z);
  CarvingOperation(carving_op_type_e type, double x, double y);
  CarvingOperation(carving_op_type_e type, double z);
  CarvingOperation(carving_op_type_e type, double x, double y, double i, double j, double p, bool ccw);

  CarvingOperation();	// declated but not definied (disallow default constructor)
  carving_op_type_e type;
  mutable double x, y, z;
  mutable double i, j;		// Arc related values; I: X offset, J: Y offset
  mutable double p;			// Arc related values; P: number of turns
  mutable bool ccw;			// Arc related values; counter-clockwise arc (default: false)
};
std::ostream &operator<<(std::ostream &stream, const CarvingOperation &op);

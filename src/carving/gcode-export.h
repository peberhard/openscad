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

#include "module.h"
#include "node.h"
#include "polyset.h"
#include "evalcontext.h"
#include "Polygon2d.h"
#include "builtin.h"
#include "printutils.h"
#include "context.h"
#include "calc.h"
#include "mathc99.h"
#include "visitor.h"

#include "carving/carving.h"
#include "carving/carving-settings.h"
#include "carving/carving-node.h"
#include "carving/carving-module.h"
#include "carving/carving-operation.h"

#include <assert.h>
#include <errno.h>

#include <sstream>
#include <map>

#include <boost/math/special_functions/fpclassify.hpp>
using boost::math::isfinite;

/* Traverse the node tree and generate a tree of G-Code operations.
 */

class GCodeExportVisitor: public Visitor
{
public:
  GCodeExportVisitor(const AbstractNode *root);
  virtual ~GCodeExportVisitor();
  virtual Response visit(class State &state, const class AbstractNode &node);
  virtual Response visit(class State &state, const class TransformNode &node);
  virtual Response visit(class State &state, const class LeafNode &node);
  virtual Response visit(class State &state, const class CsgNode &node);

  virtual Response visit(class State &state, const class CarvingDrillNode &node);
  virtual Response visit(class State &state, const class CarvingWorkpieceNode &node);
  virtual Response visit(class State &state, const class CarvingPath2dNode &node);
  virtual Response visit(class State &state, const class CarvingSliceNode &node);
  virtual Response visit(class State &state, const class CarvingReverseNode &node);
  virtual Response visit(class State &state, const class CarvingOperationNode &node);
  void fastPositioning(const class AbstractNode &node, double x, double y, double z);
  void execute();
  void toNgc(std::ostream &stream);
  static shared_ptr<class GCode> getCache(const AbstractNode *node);
  static void clearCache();
private:
  static void mergeGCodeChildren(const AbstractNode &node, shared_ptr<GCode> gc);
  const AbstractNode *root;
  Traverser *traverser;
  std::string material_name;
  shared_ptr<const CarvingTool> current_tool;
  shared_ptr<const CarvingToolSpeed> current_tool_speed;
  shared_ptr<const CarvingTool> previous_tool;
  shared_ptr<const GCode> previous_move;
  double current_thickness;
  double current_z;
  double current_feed_rate;

  static std::map<const AbstractNode *, shared_ptr<class GCode> > cache;
};

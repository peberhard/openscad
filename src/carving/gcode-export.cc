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

#include "state.h"
#include "transformnode.h"
#include "carving/carving.h"
#include "carving/gcode.h"
#include "carving/gcode-export.h"
#include "carving/carving-geometry.h"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/foreach.hpp>

#define FAST_POSITIONNING_EPSILON 0.2

#define IGNORE_NODE_IF_BACKGROUND() {                                                                       \
  if(node.modinst->isBackground()) {                                                                        \
    if(state.isPostfix()) {                                                                                 \
      shared_ptr<GCode> gc_ptr = make_shared<GContainer>("ignoring background node " + node.toString());    \
      PRINTDB("ignoring background node %s", node);                                                         \
      cache[&node] = gc_ptr;                                                                                \
    }                                                                                                       \
    return PruneTraversal;                                                                                  \
  }                                                                                                         \
}

/* GCodeExportVisitor */

std::map<const AbstractNode *, shared_ptr<class GCode> > GCodeExportVisitor::cache;

/*static*/ shared_ptr<class GCode> GCodeExportVisitor::getCache(const AbstractNode *node)
{
  assert(node);
  assert(GCodeExportVisitor::cache.count(node));
  return GCodeExportVisitor::cache.at(node);
}
/*static*/ void GCodeExportVisitor::clearCache()
{
  GCodeExportVisitor::cache.clear();
}

GCodeExportVisitor::GCodeExportVisitor(const AbstractNode *root) :
    root(root), material_name(""), /*workpiece_thickness(nan("")),*/ current_thickness(nan("")), current_z(nan("")),
        current_feed_rate(nan(""))
{
  traverser = new Traverser(*this, *this->root, Traverser::PRE_AND_POSTFIX);
}

GCodeExportVisitor::~GCodeExportVisitor()
{
  delete traverser;
}

void GCodeExportVisitor::execute()
{
  traverser->execute();
}

void GCodeExportVisitor::toNgc(std::ostream &stream)
{
  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter;
  iter = GCodeExportVisitor::cache.find(root);
  assert(GCodeExportVisitor::cache.find(root) != GCodeExportVisitor::cache.end());
  shared_ptr<GCode> gc_ptr = iter->second;
  PRINTDB("GCodeExportVisitor::toNgc %s", gc_ptr->toStr());
  gc_ptr->toNgc(stream);
}


Response GCodeExportVisitor::visit(class State &state, const class AbstractNode &node)
{
//  IGNORE_NODE_IF_BACKGROUND();
  PRINTDB("visit AbstractNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  if (state.isPrefix()) {
    return ContinueTraversal;
  }

  // postfix
  assert(GCodeExportVisitor::cache.find(&node) == GCodeExportVisitor::cache.end());

  shared_ptr<GCode> gc = make_shared<GContainer>(node.toString());

  if (state.parent() == NULL) {	// root node begin program
    gc->children.push_back(make_shared<G21>());
    gc->children.push_back(make_shared<G90>());
    gc->children.push_back(make_shared<G90_1>());
    gc->children.push_back(make_shared<G64>());
    gc->children.push_back(make_shared<G40>());
    gc->children.push_back(make_shared<G17>());
  }

  // add children
  mergeGCodeChildren(node, gc);

  if (state.parent() == NULL) {	// root node end program
    gc->children.push_back(make_shared<G49>());
    gc->children.push_back(make_shared<G0>(Carving::instance()->getSettings()->getToolChangeHeight()));
    gc->children.push_back(make_shared<M2>());
  }

  GCodeExportVisitor::cache[&node] = gc;

  return ContinueTraversal;
}

Response GCodeExportVisitor::visit(class State &state, const TransformNode &node)
{
//  IGNORE_NODE_IF_BACKGROUND();
  PRINTDB("visit TransformNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  if (state.isPrefix()) {
    return ContinueTraversal;
  }

  // postfix
  assert(GCodeExportVisitor::cache.find(&node) == GCodeExportVisitor::cache.end());

  shared_ptr<GCode> gc_ptr = make_shared<GContainer>(node.toString());
  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter;
  BOOST_FOREACH(AbstractNode *child_node, node.children) {
    iter = GCodeExportVisitor::cache.find(child_node);
    if (iter == GCodeExportVisitor::cache.end()) {
      continue;
    }
    shared_ptr<GCode> child_gc = iter->second;
    gc_ptr->children.push_back(child_gc);
  }

  GCodeExportVisitor::cache[&node] = gc_ptr;

  return ContinueTraversal;
}


Response GCodeExportVisitor::visit(class State &state, const class LeafNode &node)
{
  if (const CarvingOperationNode* op_node = dynamic_cast<const CarvingOperationNode*>(&node)) {
    return visit(state, *op_node);
  }
  if (const CarvingDrillNode* drill_node = dynamic_cast<const CarvingDrillNode*>(&node)) {
    return visit(state, *drill_node);
  }
  PRINTDB("visit LeafNode ignoring node %s", node.toString());
  return ContinueTraversal;
}


Response GCodeExportVisitor::visit(class State &state, const class CsgNode &node)
{
  if (const CarvingWorkpieceNode* workpiece_node = dynamic_cast<const CarvingWorkpieceNode*>(&node)) {
    return visit(state, *workpiece_node);
  }
  if (const CarvingPath2dNode* path2d_node = dynamic_cast<const CarvingPath2dNode*>(&node)) {
    return visit(state, *path2d_node);
  }
  if (const CarvingSliceNode* slice_node = dynamic_cast<const CarvingSliceNode*>(&node)) {
    return visit(state, *slice_node);
  }
//  if (const CarvingReverseNode* rev_node = dynamic_cast<const CarvingReverseNode*>(&node)) {
//    return visit(state, *rev_node);
//  }

  // other CsgNode, just add children
  shared_ptr<GCode> gc = make_shared<GContainer>(node.toString());
  mergeGCodeChildren(node, gc);
  GCodeExportVisitor::cache[&node] = gc;
  return ContinueTraversal;
}


void GCodeExportVisitor::fastPositioning(const class AbstractNode &node, double x, double y, double z)
{
  shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();
  shared_ptr<GCode> gc = make_shared<GContainer>(node.toString());
  if (this->previous_tool != this->current_tool) {
    PRINTDB("visit CarvingDrillNode changeTool previous_tool %p != current_tool %p",
        this->previous_tool % this->current_tool);
    this->previous_tool = this->current_tool;
    // Tool change
    // http://linuxcnc.org/docs/html/gcode/other-code.html#sec:T-Select-Tool
    // http://linuxcnc.org/docs/html/gcode/m-code.html#sec:M6-Tool-Change
    // http://linuxcnc.org/docs/html/gui/axis.html#sec:Manual-Tool-Change
    // turn tool length compensation off
    gc->children.push_back(make_shared<G49>());
    gc->children.push_back(make_shared<G0>(Carving::instance()->getSettings()->getToolChangeHeight()));
    gc->children.push_back(make_shared<TM6>(this->current_tool));
    gc->children.push_back(make_shared<G43>());               // Z offset relative to new tool

    gc->children.push_back(make_shared<M3>(this->current_tool_speed->getSpindleSpeed()));

    // Issue: Visual representation of first G0 move in LinuxCNC is not visible.
    // Workaround: Add a G1 on current position as explained in LinuxCNC documentation:
    // http://linuxcnc.org/docs/html/gui/axis.html#sec:Manual-Tool-Change
    // fix to show G0 rapid move in linuxCNC after tool change (note: empty cctx)
    gc->children.push_back(make_shared<G0>(s->getClearanceHeight()));
    // set required feedrate for G1 (even if no move)
    if (this->current_feed_rate != this->current_tool_speed->getVerticalFeedrate()) {
      this->current_feed_rate = this->current_tool_speed->getVerticalFeedrate();
      gc->children.push_back(make_shared<F>(this->current_feed_rate));
    }
    gc->children.push_back(make_shared<G1>(s->getClearanceHeight()));
  }
  gc->children.push_back(make_shared<G0>(Carving::instance()->getSettings()->getClearanceHeight()));
  gc->children.push_back(make_shared<G0>(x, y));
  gc->children.push_back(make_shared<G0>(z));
  GCodeExportVisitor::cache[&node] = gc;
}

/*static*/ void GCodeExportVisitor::mergeGCodeChildren(const AbstractNode &node, shared_ptr<GCode> gc)
{
  // add gc children to gc
  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter;
  BOOST_FOREACH(AbstractNode *child_node, node.children) {
    iter = GCodeExportVisitor::cache.find(child_node);
    if (iter == GCodeExportVisitor::cache.end()) {
      continue;
    }
    shared_ptr<GCode> child_gc = iter->second;
    gc->children.push_back(child_gc);
  }
}

Response GCodeExportVisitor::visit(class State &state, const CarvingWorkpieceNode &node)
{
  PRINTDB("visit CarvingWorkpieceNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  if (state.isPrefix()) {
    this->material_name = node.getMaterialName();
  } else { // postfix
    this->material_name = "";
    shared_ptr<GCode> gc = make_shared<GContainer>(node.toString());
    mergeGCodeChildren(node, gc);
    GCodeExportVisitor::cache[&node] = gc;
  }
  return ContinueTraversal;
}


Response GCodeExportVisitor::visit(class State &state, const class CarvingDrillNode &node)
{
  PRINTDB("visit CarvingDrillNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  assert(node.getChildren().empty());

  shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();

  if (state.isPrefix()) {
    this->current_tool = s->getTool(node.getToolName());
    this->current_tool_speed = s->getToolSpeed(node.getToolName(), this->material_name);
    fastPositioning(node, node.getX(), node.getY(), DEFAULT_Z + FAST_POSITIONNING_EPSILON);
    return ContinueTraversal;
  }

  // postfix

  shared_ptr<GCode> gc_ptr;
  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter = GCodeExportVisitor::cache.find(&node);
  assert(iter != GCodeExportVisitor::cache.end());
  gc_ptr = iter->second;

  shared_ptr<const CarvingTool> drill_tool = s->getTool(node.getToolName());
  shared_ptr<const CarvingToolSpeed> drill_tool_speed = s->getToolSpeed(node.getToolName(), this->material_name);

  // drill only

  // set feed rate
  if (this->current_feed_rate != drill_tool_speed->getVerticalFeedrate()) {
    this->current_feed_rate = drill_tool_speed->getVerticalFeedrate();
    gc_ptr->children.push_back(make_shared<F>(this->current_feed_rate));
  }

  // TODO Carving: tool.head_end is ato be added to thickness in order to get though properly
  gc_ptr->children.push_back(make_shared<G1>(-node.getThickness()));
  gc_ptr->children.push_back(make_shared<G0>(s->getClearanceHeight()));
  GCodeExportVisitor::cache[&node] = gc_ptr;
  return ContinueTraversal;
}


Response GCodeExportVisitor::visit(class State &state, const class CarvingPath2dNode &node)
{
  PRINTDB("visit CarvingPath2dNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();

  if (state.isPrefix()) {
    this->current_tool = s->getTool(node.getToolName());
    this->current_tool_speed = s->getToolSpeed(node.getToolName(), this->material_name);
    isfinite(node.getThickness());
    this->current_thickness = node.getThickness();
    this->current_z = DEFAULT_Z;
    this->current_feed_rate = nan("");
    fastPositioning(node, node.getX(), node.getY(), DEFAULT_Z + FAST_POSITIONNING_EPSILON);
    return ContinueTraversal;
  }

  // state is postfix

  // fastPositioning has been added, check that we find it.
  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter = GCodeExportVisitor::cache.find(&node);
  assert(iter != GCodeExportVisitor::cache.end());
  shared_ptr<GCode> gc_ptr = iter->second;

  assert(!node.getChildren().empty());

  // add G-Code children in child_cont_ptr
  mergeGCodeChildren(node, gc_ptr);

  // back to clearance height
  gc_ptr->children.push_back(make_shared<G0>(s->getClearanceHeight()));

  GCodeExportVisitor::cache[&node] = gc_ptr;
  this->current_tool.reset();
  this->current_tool_speed.reset();
  this->current_thickness = nan("");
  this->current_z = nan("");
  this->current_feed_rate = nan("");

  return ContinueTraversal;
}



Response GCodeExportVisitor::visit(class State &state, const class CarvingSliceNode &node)
{
  PRINTDB("visit CarvingSliceNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();

  if (state.isPrefix()) {
    return ContinueTraversal;
  }

  // state is postfix

  std::map<const AbstractNode *, shared_ptr<class GCode> >::iterator iter = GCodeExportVisitor::cache.find(&node);
  assert(iter == GCodeExportVisitor::cache.end());
  shared_ptr<GCode> gc = make_shared<GContainer>(node.toString());

  // add G-Code children in child_cont_ptr
  mergeGCodeChildren(node, gc);

  GCodeExportVisitor::cache[&node] = gc;

  return ContinueTraversal;
}

Response GCodeExportVisitor::visit(class State &state, const class CarvingReverseNode &node)
{
  PRINTDB("visit CarvingReverseNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));

  assert(!"should not be called");

  return ContinueTraversal;
}


Response GCodeExportVisitor::visit(class State &state, const class CarvingOperationNode &node)
{
  PRINTDB("visit CarvingOperationNode '%s' %s", node % (state.isPrefix() ? "prefix" : "postfix"));
  assert(node.getOp());

  if (state.isPrefix()) {
    return ContinueTraversal;
  }

  // postfix

  assert(GCodeExportVisitor::cache.find(&node) == GCodeExportVisitor::cache.end());
  shared_ptr<GCode> gc;

  CarvingOperation *op = node.getOp();
  switch (op->getType()) {
  case CARVING_OP_PATH2D:
    assert(isfinite(op->getX()) && isfinite(op->getX()));
    gc = make_shared<G1>(node.getOp()->getZ());
    break;
  case CARVING_OP_LINEAR_MOVE_XY:
    assert(isfinite(op->getX()) && isfinite(op->getX()));
    gc = make_shared<G1>(op->getX(), op->getY());
    break;
  case CARVING_OP_ARC_MOVE_XY:
    assert((isfinite(op->getX()) && isfinite(op->getX()) && isfinite(op->getI()) && isfinite(op->getJ())));
    assert(!(op->getX() == op->getI() && op->getY() == op->getJ()));
    if (!op->getCcw()) {
      gc = make_shared<G2>(op->getX(), op->getY(), op->getI(), op->getJ(), op->getP());
    } else {
      gc = make_shared<G3>(op->getX(), op->getY(), op->getI(), op->getJ(), op->getP());
    }
    break;
  default:
    PRINTB("ERROR: Carving: Unable to export child, unknown type, ignoring node %s ", *op);
    return ContinueTraversal;
  }
  assert(gc.get() != NULL);

  // handle feed rate separately
  shared_ptr<GCode> gcf;
  switch (op->getType()) {
  case CARVING_OP_PATH2D:
    if (this->current_feed_rate != this->current_tool_speed->getVerticalFeedrate()) {
      this->current_feed_rate = this->current_tool_speed->getVerticalFeedrate();
      gcf = make_shared<F>(this->current_feed_rate);
    }
    break;
  case CARVING_OP_LINEAR_MOVE_XY:
    if (this->current_feed_rate != this->current_tool_speed->getHorizontalFeedrate()) {
      this->current_feed_rate = this->current_tool_speed->getHorizontalFeedrate();
      gcf = make_shared<F>(this->current_feed_rate);
    }
    break;
  case CARVING_OP_ARC_MOVE_XY:
    if (this->current_feed_rate != this->current_tool_speed->getArcFeedrate()) {
      this->current_feed_rate = this->current_tool_speed->getArcFeedrate();
      gcf = make_shared<F>(this->current_feed_rate);
    }
    break;
  default:
    PRINTB("ERROR: Carving: Unable to export child, unknown type, ignoring node %s ", *op);
    return ContinueTraversal;
  }
  if (gcf) {
    shared_ptr<GContainer> gc_cont(new GContainer(""));
    gc_cont->children.push_back(gcf);
    gc_cont->children.push_back(gc);
    gc = gc_cont;
  }

  this->previous_move = gc;
  GCodeExportVisitor::cache[&node] = gc;
  PRINTDB("visit CarvingOperationNode added gc %s", gc->toString());
  return ContinueTraversal;
}


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

#include "node.h"
#include "state.h"
#include "linalg.h"
#include "printutils.h"
#include "transformnode.h"
#include "carving/carving.h"
#include "carving/carving-node.h"
#include "carving/carving-operation.h"
#include "carving/carving-settings.h"
#include "carving/carving-update-visitor.h"
#include "carving/carving-geometry.h"
#include <vector>
#include <list>
#include <boost/foreach.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

using boost::math::isfinite;

static void reverse_children(const AbstractNode *const_node)
{
  AbstractNode *node = const_cast<AbstractNode *>(const_node);
  std::reverse(node->children.begin(), node->children.end());
}


/*
 * GCodeUpdateCCtxVisitor
 */

CarvingUpdateVisitor::CarvingUpdateVisitor(const AbstractNode *root) :
    root(root), in_reverse_node(false), current_thickness(nan("")), current_z(nan("")), previous_op_node(NULL),
    previous_op_node_rev(NULL), first_op_in_slice(false)
{
  assert(this->root);
  traverser = new Traverser(*this, *this->root, Traverser::PRE_AND_POSTFIX);
}

CarvingUpdateVisitor::~CarvingUpdateVisitor()
{
  delete traverser;
}

void CarvingUpdateVisitor::execute()
{
  traverser->execute();
}

Response CarvingUpdateVisitor::visit(class State &state, const class AbstractNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (state.isPrefix()) {
    if (in_reverse_node) {
      reverse_children(&node);
    }
  } else {

  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const class CsgNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (const CarvingWorkpieceNode* workpiece_node = dynamic_cast<const CarvingWorkpieceNode*>(&node)) {
    return visit(state, *workpiece_node);
  }

  if (const CarvingPath2dNode* path2d_node = dynamic_cast<const CarvingPath2dNode*>(&node)) {
    return visit(state, *path2d_node);
  }

  if (const CarvingSliceNode* slice_node = dynamic_cast<const CarvingSliceNode*>(&node)) {
    return visit(state, *slice_node);
  }

  if (const CarvingReverseNode* reverse_node = dynamic_cast<const CarvingReverseNode*>(&node)) {
    return visit(state, *reverse_node);
  }

  if (state.isPrefix()) {
    if (in_reverse_node) {
      reverse_children(&node);
    }
  } else {

  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const TransformNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  // if not inside workpiece
  if (this->material_name.empty()) {
    return ContinueTraversal;
  }

  if (state.isPrefix()) {
    state.setMatrix(state.matrix() * node.matrix);
    if (in_reverse_node) {
      reverse_children(&node);
    }
  } else { // postfix
    // FIXME Carving: Ugly, now that TransformNode has been applied, it must not be applied anymore
    TransformNode *noconst_node = const_cast<TransformNode*>(&node);
    noconst_node->matrix = noconst_node->matrix.Identity();
  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const class LeafNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  const CarvingOperationNode* op_node = dynamic_cast<const CarvingOperationNode*>(&node);
  if (op_node) {
    return visit(state, *op_node);
  }

  const CarvingDrillNode* drill_node = dynamic_cast<const CarvingDrillNode*>(&node);
  if (drill_node) {
    return visit(state, *drill_node);
  }

  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const class CarvingWorkpieceNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (state.isPrefix()) {
    this->material_name = node.getMaterialName();
    state.setMatrix(Transform3d::Identity());
  } else { // postfix
    this->material_name = "";
  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const class CarvingDrillNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  assert(!in_reverse_node);
  if (state.isPrefix()) {
    assert(isfinite(node.getX()) && isfinite(node.getY()));
    Eigen::Vector2d v(node.getX(), node.getY());
    Transform2d matrix2d(transform3d_to_transform2d(state.matrix()));
    v = matrix2d * v;
    node.setX(v.x());
    node.setY(v.y());
  } else { // postfix

  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const CarvingPath2dNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  assert(!in_reverse_node);
  if (state.isPrefix()) {
    // reset transform matrix to get vectors relative to make children local coordinates relative to path2d.
    shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();
    this->current_tool = s->getTool(node.getToolName());
    this->current_tool_speed = s->getToolSpeed(node.getToolName(), this->material_name);
    assert(isfinite(node.getThickness()));
    this->current_thickness = node.getThickness();

    assert(isfinite(node.getX()) && isfinite(node.getY()));
    CarvingPath2dNode* noconst_node = const_cast<CarvingPath2dNode*>(&node);
    Transform2d matrix2d(transform3d_to_transform2d(state.matrix()));
    Eigen::Vector2d v(noconst_node->getX(), noconst_node->getY());
    v = matrix2d * v;
    noconst_node->setX(v.x());
    noconst_node->setY(v.y());
    if (isfinite(noconst_node->getPosX()) && isfinite(noconst_node->getPosY())) {
      Eigen::Vector2d v2(noconst_node->getPosX(), noconst_node->getPosY());
      v2 = matrix2d * v2;
      noconst_node->setPosX(v2.x());
      noconst_node->setPosY(v2.y());
    }
  } else { // postfix
    this->current_tool.reset();
    this->current_tool_speed.reset();
    this->current_thickness = nan("");
  }
  return ContinueTraversal;
}


Response CarvingUpdateVisitor::visit(class State &state, const class CarvingSliceNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (state.isPrefix()) {
    assert(!this->in_reverse_node);
    shared_ptr<CarvingSettings> s = Carving::instance()->getSettings();

    this->in_reverse_node = node.getReverse();
    if (this->in_reverse_node) {
      reverse_children(&node);
      previous_op_node_rev = NULL;
      first_op_in_slice = true;
    } else {
        // no specific code in non reverse
    }
    this->current_z = node.getZ();

  } else { // postfix

    assert(this->in_reverse_node == node.getReverse());
    if (this->in_reverse_node) {
      this->in_reverse_node = false;
      previous_op_node_rev = NULL;
    }
    this->current_z = nan("");
  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const class CarvingReverseNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (state.isPrefix()) {
    in_reverse_node = !in_reverse_node;
    if (in_reverse_node) {
      reverse_children(&node);
    }
  } else { // postfix
    in_reverse_node = !in_reverse_node;
  }
  return ContinueTraversal;
}

Response CarvingUpdateVisitor::visit(class State &state, const CarvingOperationNode &node)
{
//  PRINTDB("visit class %s prefix %d, current_tool %p", typeid(node).name() % state.isPrefix() % current_tool.get());
  if (state.isPrefix()) {
    if (in_reverse_node) {
      CarvingOperation *next_previous_op_node_rev = node.getOp();
      CarvingOperationNode* noconst_node = const_cast<CarvingOperationNode*>(&node);
      CarvingOperation *rev_op = NULL;
      if (previous_op_node_rev == NULL) {
        if (first_op_in_slice) {
          rev_op = CarvingOperation::newGCodeOpPath2d(node.getOp()->getX(), node.getOp()->getY(), this->current_z);
        } else {
          rev_op = CarvingOperation::newGCodeOpLinearMoveXY(node.getOp()->getX(), node.getOp()->getY());
        }
      } else {
        switch (previous_op_node_rev->getType()) {
        case CARVING_OP_LINEAR_MOVE_XY:
          rev_op = CarvingOperation::newGCodeOpLinearMoveXY(node.getOp()->getX(), node.getOp()->getY());
          break;
        case CARVING_OP_ARC_MOVE_XY:
          rev_op = CarvingOperation::newGCodeOpArcMoveXY(node.getOp()->getX(), node.getOp()->getY(),
              previous_op_node_rev->getI(), previous_op_node_rev->getJ(), previous_op_node_rev->getP(),
              !previous_op_node_rev->getCcw());
          break;
        default:
          PRINTB("ERROR: Carving: Unable to reverse %s, old %s", node.toString() % previous_op_node_rev->toString());

        }
      }
      if (first_op_in_slice) {
        first_op_in_slice = false;
      }
      if (rev_op) {
        noconst_node->setOp(rev_op);
      }
      previous_op_node_rev = next_previous_op_node_rev;
    } else {
      // not reverse
    }
    // common to reverse and non-reverse op nodes
    node.setOperationContext(current_tool, current_tool_speed, current_thickness, previous_op_node);
    node.getOp()->transform(transform3d_to_transform2d(state.matrix()));
    previous_op_node = &node;
  } else {
    // postfix
  }
  return ContinueTraversal;
}



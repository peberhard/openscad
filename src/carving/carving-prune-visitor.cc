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
#include "carving/carving-prune-visitor.h"
#include "carving/carving-geometry.h"
#include <vector>
#include <list>
#include <boost/foreach.hpp>


/*
 * GCodeUpdateCCtxVisitor
 */

CarvingPruneVisitor::CarvingPruneVisitor(const AbstractNode *root) : root(root)
{
  assert(this->root);
  traverser = new Traverser(*this, *this->root, Traverser::PRE_AND_POSTFIX);
}

CarvingPruneVisitor::~CarvingPruneVisitor()
{
  delete traverser;
}

void CarvingPruneVisitor::execute()
{
  traverser->execute();
}

void CarvingPruneVisitor::removeBadChildren(const AbstractNode *const_node)
{
  AbstractNode *node = const_cast<AbstractNode *>(const_node);
  std::list<AbstractNode*> removal_list = this->removal_map[node];
  if (removal_list.empty()) {
    return;
  }
  PRINTDB("%d bad children to be removed from %s", removal_list.size() % *node);
  BOOST_FOREACH(AbstractNode *bad_child, removal_list) {
    std::vector<AbstractNode*>::iterator it;
    for (it = node->children.begin(); it != node->children.end(); ++it) {
      if (bad_child == *it) {
        break;
      }
    }
    assert(it != node->children.end());
    PRINTDB("removing bad node %s", **it);
    node->children.erase(it);
    free(bad_child);
  } // end for each bad_child in removal_list
  this->removal_map[node].clear();
}

Response CarvingPruneVisitor::visit(class State &state, const class AbstractNode &node)
{
  if (state.isPrefix()) {
      // nothing to do
  } else {
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CsgNode &node)
{
  if (const CarvingWorkpieceNode* workpiece_node = dynamic_cast<const CarvingWorkpieceNode*>(&node)) {
    return visit(state, *workpiece_node);
  }

  if (const CarvingPath2dNode* path2d_node = dynamic_cast<const CarvingPath2dNode*>(&node)) {
    return visit(state, *path2d_node);
  }

  if (const CarvingReverseNode* reverse_node = dynamic_cast<const CarvingReverseNode*>(&node)) {
    return visit(state, *reverse_node);
    // still do removeBadChildren below
  }

  if (state.isPrefix()) {
    // nothing to do
  } else { // postfix
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class TransformNode &node)
{
  //PRINTDB("GCodeUpdateCCtxVisitor::visit visited class %s prefix %d, in_path2d %d",
  //        typeid(node).name() % state.isPrefix() % in_path2d);
  if (state.isPrefix()) {
    // nothing to do
  } else { // postfix
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class LeafNode &node)
{
//	PRINTDB("GCodeUpdateCCtxVisitor::visit visited class %s prefix %d", typeid(node).name() % state.isPrefix());
  if (const CarvingOperationNode* op_node = dynamic_cast<const CarvingOperationNode*>(&node)) {
    return visit(state, *op_node);
  }

  if (const CarvingDrillNode* drill_node = dynamic_cast<const CarvingDrillNode*>(&node)) {
    return visit(state, *drill_node);
  }

  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CarvingWorkpieceNode &node)
{
  PRINTDB("visit %s %s", node.name() % (state.isPrefix() ? "prefix" : "    suffix"));
  if (state.isPrefix()) {
    if (!this->call_stack.empty()) {
      PRINTB("WARNING: Carving: %s() shall be a child of no one, parent is %s()", node.name() % this->call_stack.back());
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *) &node);
      this->call_stack.push_back(node.name());
      return PruneTraversal;    // abort children but run postfix anyway
    }
    this->call_stack.push_back(node.name());

  } else { // postfix

    assert(this->call_stack.back() == node.name());
    this->call_stack.pop_back();
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CarvingDrillNode &node)
{
  //PRINTDB("visit %s %s", node.name() % (state.isPrefix() ? "prefix" : "    suffix"));
  if (state.isPrefix()) {
    if (this->call_stack.empty() || this->call_stack.back() != CARVING_WORKPIECE_STR) {
      PRINTB("WARNING: Carving: %s() shall be a child of carving_workpiece(), parent is %s",
          node.name() % (this->call_stack.empty() ? "(empty stack)" : this->call_stack.back()));
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *)&node);
      return PruneTraversal;    // abort children but run postfix anyway
    }
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CarvingPath2dNode &node)
{
  if (state.isPrefix()) {
    if (this->call_stack.empty() || this->call_stack.back() != CARVING_WORKPIECE_STR) {
      PRINTB("WARNING: Carving: %s() shall be a child of carving_workpiece(), parent is %s",
          node.name() % (this->call_stack.empty() ? "(empty stack)" : this->call_stack.back()));
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *)&node);
      this->call_stack.push_back(node.name());
      return PruneTraversal;
    }
    this->call_stack.push_back(node.name());

    if (node.getChildren().empty()) {
      PRINTB("WARNING: Carving: %s() shall have at least one child", node.name());
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *)&node);
    }

} else { // postfix

    assert(this->call_stack.back() == node.name());
    this->call_stack.pop_back();
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CarvingReverseNode &node)
{
  if (state.isPrefix()) {
    if (node.getChildren().empty()) {
      PRINTB("WARNING: Carving: %s() shall have at least one child", node.name());
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *)&node);
    }
  } else { // postfix
    removeBadChildren(&node);
  }
  return ContinueTraversal;
}

Response CarvingPruneVisitor::visit(class State &state, const class CarvingOperationNode &node)
{
  //  PRINTDB("GCodeUpdateCCtxVisitor::visit visited class %s prefix %d", typeid(node).name() % state.isPrefix());
  if (state.isPrefix()) {
    if (this->call_stack.empty() || this->call_stack.back() != CARVING_PATH2D_STR) {
      PRINTB("WARNING: Carving: %s() shall be a child of carving_path2d(), parent is %s",
          node.name() % (this->call_stack.empty() ? "(empty stack)" : this->call_stack.back()));
      assert(state.parent() != NULL);
      this->removal_map[state.parent()].push_back((AbstractNode *)&node);
      return PruneTraversal;    // abort children but run postfix anyway
    }
  }
  return ContinueTraversal;
}

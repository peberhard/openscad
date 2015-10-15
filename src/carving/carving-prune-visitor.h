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

#include "state.h"
#include "visitor.h"
#include "carving/carving-settings.h"

#include <map>
#include <list>
#include <boost/shared_ptr.hpp>

using boost::shared_ptr;


/* Traverse the node tree and prune invalid branches and leaves
 */

class CarvingPruneVisitor: public Visitor
{
public:
  CarvingPruneVisitor(const AbstractNode *root);
  virtual ~CarvingPruneVisitor();
  virtual Response visit(class State &state, const class AbstractNode &node);
  virtual Response visit(class State &state, const class CsgNode &node);
  virtual Response visit(class State &state, const class TransformNode &node);
  virtual Response visit(class State &state, const class LeafNode &node);

  virtual Response visit(class State &state, const class CarvingWorkpieceNode &node);
  virtual Response visit(class State &state, const class CarvingDrillNode &node);
  virtual Response visit(class State &state, const class CarvingPath2dNode &node);
  virtual Response visit(class State &state, const class CarvingReverseNode &node);
  virtual Response visit(class State &state, const class CarvingOperationNode &node);
  void execute();
private:
  const AbstractNode *root;
  Traverser *traverser;

  /* used for cleaning the tree on postfix */
  std::list<std::string> call_stack;
  std::map<const AbstractNode*, std::list<AbstractNode*> > removal_map;
  void removeBadChildren(const AbstractNode *const_node);
};

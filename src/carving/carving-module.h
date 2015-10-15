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

#include "node.h"
#include "module.h"
#include "carving/carving-node.h"
#include <string>
#include <list>

class CarvingModule: public AbstractModule
{
public:
  CarvingModule(carving_type_e type);
  ~CarvingModule();
  virtual AbstractNode *instantiate(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;

private:
  carving_type_e type;
  AbstractNode *instantiateSettingsNode(const Context *ctx, const ModuleInstantiation *inst,
      EvalContext *evalctx) const;
  AbstractNode *instantiateToolNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateToolSpeedNode(const Context *ctx, const ModuleInstantiation *inst,
      EvalContext *evalctx) const;
  AbstractNode *instantiateWorkpieceNode(const Context *ctx, const ModuleInstantiation *inst,
      EvalContext *evalctx) const;
  AbstractNode *instantiatePartNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateDrillNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiatePath2dNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateMoveNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateArcMoveNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateReverseNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;
  AbstractNode *instantiateAssemblyNode(const Context *ctx, const ModuleInstantiation *inst, EvalContext *evalctx) const;

  static std::string workpiece_material_name;   // later: not thread-safe :'(
  static double workpiece_thickness;
  static double current_z;
};

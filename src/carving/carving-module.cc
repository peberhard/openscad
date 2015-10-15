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

#include "evalcontext.h"
#include "printutils.h"
#include "carving/carving.h"
#include "carving/carving-node.h"
#include "carving/carving-module.h"
#include "carving/carving-operation.h"
#include "csgnode.h"
#include <algorithm>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/math/special_functions/fpclassify.hpp>

using namespace boost::assign; // bring 'operator+=()' into scope
using boost::math::isfinite;

/*
 * Context helper function to extract variables
 */

static int ctx_get_vect2d(Context &c, const std::string &module_name, const std::string &variable_name, double &x,
                          double &y, bool required = true)
{
  ValuePtr value = c.lookup_variable(variable_name);
  double tmpx, tmpy;
  if ((value->type() != Value::VECTOR || !value->getVec2(tmpx, tmpy))) {
    if (required) {
      PRINTB("WARNING: Carving: ignoring %s() due to invalid parameters: %s='%s' is not a 2d vector.",
          module_name % variable_name % *value);
    }
    return -1;
  } else {
    x = tmpx;
    y = tmpy;
  }
  return 0;
}

static double ctx_get_double(Context &c, const char *variable_name, double failback = nan(""))
{
  const ValuePtr var = c.lookup_variable(variable_name);
  if (var->type() == Value::NUMBER) {
    return var->toDouble();
  } else {
    return failback;
  }
}

static int ctx_get_double_gt0(Context &c, const std::string &module_name, const char *variable_name, double &value,
    bool required = true)
{
  double tmp = ctx_get_double(c, variable_name, nan(""));
  if ((!isfinite(tmp) || tmp <= 0)) {
    if (required) {
      PRINTB("WARNING: Carving: ignoring %s() due to invalid parameters: %s='%s' is not a double.",
          module_name % variable_name % *c.lookup_variable(variable_name));
      return -1;
    }
  } else {
    value = tmp;
  }
  return 0;
}

static int ctx_get_int_gt0(Context &c, const std::string &module_name, const char *variable_name, int &value,
                              bool required = true)
{
  int tmp = ctx_get_double(c, variable_name, -1);
  if (tmp <= 0) {
    if (required) {
      PRINTB("WARNING: Carving: ignoring %s() due to invalid parameters: %s='%s' is not a double.",
          module_name % variable_name % *c.lookup_variable(variable_name));
      return -1;
    }
  } else {
    value = tmp;
  }
  return 0;
}

static int ctx_get_string(Context &c, const std::string &module_name, const std::string &variable_name,
                          std::string &value, bool required = true)
{
  bool found = false;
  std::string tmp;
  ValuePtr value_ptr = c.lookup_variable(variable_name);
  if (value_ptr->isDefined()) {
    tmp = value_ptr->toString();
    if (!tmp.empty()) {
      found = true;
    }
  }

  if (!found && required) {
    PRINTB("WARNING: Carving: ignoring %s() due to invalid parameters: %s='%s' is not a string or is empty.",
            module_name % variable_name % *value_ptr);
    return -1;
  }
  value = tmp;
  return 0;
}



/*
 * CarvingModule
 */

std::string CarvingModule::workpiece_material_name = "";
double CarvingModule::workpiece_thickness = nan("");
double CarvingModule::current_z = nan("");

CarvingModule::CarvingModule(carving_type_e type) :
    type(type)
{
}

CarvingModule::~CarvingModule()
{
}

AbstractNode *CarvingModule::instantiate(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AbstractNode *node = NULL;
  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();

  // carving_settings/tool/tool_speed must be called before any other module
  if (!settings || !settings->isValid()) {
    if (this->type != CARVING_SETTINGS && this->type != CARVING_TOOL && this->type != CARVING_TOOL_SPEED) {
      PRINTB("WARNING: Carving: %s(), settings are invalid, check carving_settings(), carving_tool(), carving_tool_speed() calls.",
          inst->name());
      return NULL;
    }
  }

  switch (this->type) {
  case CARVING_SETTINGS:
    node = instantiateSettingsNode(ctx, inst, evalctx);
    break;
  case CARVING_TOOL:
    node = instantiateToolNode(ctx, inst, evalctx);
    break;
  case CARVING_TOOL_SPEED:
    node = instantiateToolSpeedNode(ctx, inst, evalctx);
    break;
  case CARVING_WORKPIECE:
    node = instantiateWorkpieceNode(ctx, inst, evalctx);
    break;
  case CARVING_DRILL:
    node = instantiateDrillNode(ctx, inst, evalctx);
    break;
  case CARVING_PATH2D:
    node = instantiatePath2dNode(ctx, inst, evalctx);
    break;
  case CARVING_LINEAR_MOVE:
    node = instantiateLinearMoveNode(ctx, inst, evalctx);
    break;
  case CARVING_ARC_MOVE:
    node = instantiateArcMoveNode(ctx, inst, evalctx);
    break;
  case CARVING_REVERSE:
    node = instantiateReverseNode(ctx, inst, evalctx);
    break;
  case CARVING_ASSEMBLY:
    node = instantiateAssemblyNode(ctx, inst, evalctx);
    break;
  case CARVING_PART:
    node = instantiatePartNode(ctx, inst, evalctx);
    break;
  default:
    PRINTB("WARNING: Carving: ignoring %s(). type=%d unknown.", inst->name() % this->type);
  }
  return node;
}

AbstractNode *CarvingModule::instantiateSettingsNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);

  args += Assignment("tool_change_height"), Assignment("clearance_height");
          /* Assignment("fn"), Assignment("fs"), Assignment("fa"); */
  c.setVariables(args, evalctx);

  double tool_change_height = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "tool_change_height", tool_change_height) < 0) {
    return NULL;
  }
  double clearance_height = ctx_get_double(c, "clearance_height", 5);
/*
  double fn = ctx_get_double(c, "fn", 8);
  double fs = ctx_get_double(c, "fs", 0);
  double fa = ctx_get_double(c, "fa", 0);
*/
  double fn = c.lookup_variable("$fn")->toDouble();
  double fa = c.lookup_variable("$fa")->toDouble();
  double fs = c.lookup_variable("$fs")->toDouble();
  //PRINTB("%s %s %s", fn % fa % fs);

  shared_ptr<CarvingSettings> s(new CarvingSettings(tool_change_height, clearance_height, fn, fs, fa));
  Carving::instance()->setSettings(s);

  return NULL;
}

AbstractNode *CarvingModule::instantiateToolNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);
  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();

  if (!settings) {
    PRINTB("WARNING: Carving: ignoring %s() because carving_settings() must be called first", inst->name());
    return NULL;
  }

  args += Assignment("tool_number"), Assignment("name"), Assignment("diameter"), Assignment("front_angle"),
          Assignment("head_end"), Assignment("info");
  c.setVariables(args, evalctx);

  int tool_number = -1;
  if (ctx_get_int_gt0(c, inst->name(), "tool_number", tool_number) < 0) {
    return NULL;
  }
  std::string name;
  if(ctx_get_string(c, inst->name(), "name", name) < 0) {
    return NULL;
  }
  std::string info;
  ctx_get_string(c, inst->name(), "info", info, false);

  double diameter = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "diameter", diameter) < 0) {
    return NULL;
  }
  double front_angle = 0;
  ctx_get_double_gt0(c, inst->name(), "front_angle", front_angle, false);
  double head_end = 0;
  ctx_get_double_gt0(c, inst->name(), "head_end", head_end, false);

  if (settings->isTool(name)) {
    PRINTB("WARNING: Carving: %s(), tool name '%s' has already been registered", inst->name() % name);
    return NULL;
  }
  // TODO Carving: check that tool number is unique
  shared_ptr<const CarvingTool> tool(new CarvingTool(tool_number, name, diameter, front_angle, head_end, info));
  PRINTDB("%s() adding tool %s", inst->name() % *tool);
  settings->setTool(tool);

  return NULL;
}

AbstractNode *CarvingModule::instantiateToolSpeedNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);

  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();

  if (!settings) {
    PRINTB("WARNING: Carving: %s() shall be preceded by carving_settings().", inst->name());
    return NULL;
  }
  if (settings->getTools().empty()) {
    PRINTB("WARNING: Carving: %s() shall be preceded by carving_tool().", inst->name());
    return NULL;
  }

  args += Assignment("tool"), Assignment("material"), Assignment("spindle_speed"), Assignment("feedrate"),
          Assignment("step_down");
  c.setVariables(args, evalctx);

  std::string tool;
  if(ctx_get_string(c, inst->name(), "tool", tool) < 0) {
    return NULL;
  }
  std::string material;
  if(ctx_get_string(c, inst->name(), "material", material) < 0) {
    return NULL;
  }
  double spindle_speed = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "spindle_speed", spindle_speed) < 0) {
    return NULL;
  }
  double feedrate = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "feedrate", feedrate) < 0) {
    return NULL;
  }
  double step_down = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "step_down", step_down) < 0) {
    return NULL;
  }

  if (!settings->isTool(tool)) {
    PRINTB("WARNING: Carving: %s(), tool name '%s' must be registered with carving_tool() first", inst->name() % tool);
    return NULL;
  }

  shared_ptr<const CarvingToolSpeed> toolspeed(
      new CarvingToolSpeed(tool, material, spindle_speed, feedrate, step_down));
  PRINTDB("%s() adding tool_speed %s", inst->name() % *toolspeed);
  settings->setToolSpeed(toolspeed);

  return NULL;
}

AbstractNode *CarvingModule::instantiateWorkpieceNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  PRINTDB("Carving: %s()", inst->name());
  AssignmentList args;
  Context c(ctx);

  inst->scope.apply(*evalctx);

  args += Assignment("v"), Assignment("material"), Assignment("thickness");
  c.setVariables(args, evalctx);

  double x = nan("");
  double y = nan("");
  if (ctx_get_vect2d(c, inst->name(), "v", x, y, true) < 0) {
    return NULL;
  }

  std::string material_name;
  if(ctx_get_string(c, inst->name(), "material", material_name) < 0) {
    return NULL;
  }
  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();
  if (!settings->isMaterial(material_name)) {
    PRINTB("WARNING: Carving: %s(), material name '%s' must be registered with carving_toolspeed() first", inst->name() % material_name);
    return NULL;
  }
  CarvingModule::workpiece_material_name = material_name;

  double thickness = nan("");
  if (ctx_get_double_gt0(c, inst->name(), "thickness", thickness) < 0) {
    return NULL;
  }
  CarvingModule::workpiece_thickness = thickness;

  CarvingWorkpieceNode *node = new CarvingWorkpieceNode(inst, material_name, x, y, thickness);

  if (!Carving::instance()->isRenderingModePath()) {
    // When rendering in result or assembly mode, the workpiece is represented by the material_name node in order to
    // make the difference between it and the carving path.
    CarvingMaterialNode *child = new CarvingMaterialNode(inst, material_name, x, y, thickness);
    node->children.push_back(child);
    PRINTDB("mode result, adding material_name %s", *child);
  }

  // finally, take care of children
  std::vector<AbstractNode *> instantiatednodes = inst->instantiateChildren(evalctx);
  node->children.insert(node->children.end(), instantiatednodes.begin(), instantiatednodes.end());

  return node;
}

AbstractNode *CarvingModule::instantiatePartNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);

  inst->scope.apply(*evalctx);

  args += Assignment("id");
  c.setVariables(args, evalctx);

  std::string id;
  if (ctx_get_string(c, inst->name(), "id", id) < 0) {
    return NULL;
  }

  CarvingPartNode *node = new CarvingPartNode(inst, id);
  return node;
}

AbstractNode *CarvingModule::instantiateDrillNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);

  inst->scope.apply(*evalctx);

  args += Assignment("tool");
  args += Assignment("v");
  args += Assignment("thickness");
  c.setVariables(args, evalctx);

  std::string tool;
  if(ctx_get_string(c, inst->name(), "tool", tool) < 0) {
    return NULL;
  }
  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();
  if (!settings->isTool(tool)) {
    PRINTB("WARNING: Carving: %s(), tool name '%s' must be registered with carving_tool() first", inst->name() % tool);
    return NULL;
  }

  if (!isfinite(workpiece_thickness)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_workpiece() module", inst->name());
    return NULL;
  }

  if (!isfinite(workpiece_thickness)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_workpiece() module", inst->name());
    return NULL;
  }

  double x = 0;
  double y = 0;
  if (ctx_get_vect2d(c, inst->name(), "v", x, y, false) < 0) {
    return NULL;
  }

  double thickness = nan("");
  ctx_get_double_gt0(c, inst->name(), "thickness", thickness, false);
  if (!isfinite(thickness)) {
    assert(isfinite(workpiece_thickness));
    thickness = workpiece_thickness;
  }
  return new CarvingDrillNode(inst, tool, x, y, thickness);
}

AbstractNode *CarvingModule::instantiatePath2dNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);

  inst->scope.apply(*evalctx);

  args += Assignment("tool");
  args += Assignment("v");
  args += Assignment("id");
  args += Assignment("pos");
  args += Assignment("thickness");
  c.setVariables(args, evalctx);

  std::string tool_name;
  if(ctx_get_string(c, inst->name(), "tool", tool_name) < 0) {
    return NULL;
  }
  shared_ptr<CarvingSettings> settings = Carving::instance()->getSettings();
  if (!settings->isTool(tool_name)) {
    PRINTB("WARNING: Carving: %s(), tool name '%s' must be registered with carving_tool() first", inst->name() % tool_name);
    return NULL;
  }

  if (!isfinite(workpiece_thickness)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_workpiece() module", inst->name());
    return NULL;
  }

  if (!isfinite(workpiece_thickness)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_workpiece() module", inst->name());
    return NULL;
  }

  double x = nan("");
  double y = nan("");
  if (ctx_get_vect2d(c, inst->name(), "v", x, y, true) < 0) {
    return NULL;
  }

  std::string id;
  ctx_get_string(c, inst->name(), "id", id, false);

  double pos_x = nan("");
  double pos_y = nan("");
  if (ctx_get_vect2d(c, inst->name(), "pos", pos_x, pos_y, false)) {
    if (!id.empty()) {
      pos_x = 0;
      pos_y = 0;
    }
  }

  double thickness = nan("");
  ctx_get_double_gt0(c, inst->name(), "thickness", thickness, false);
  if (!isfinite(thickness)) {
    assert(isfinite(workpiece_thickness));
    thickness = workpiece_thickness;
  }

  PRINTDB("Carving: %s()", inst->name());
  CarvingPath2dNode *node = new CarvingPath2dNode(inst, tool_name, x, y, thickness, id, pos_x, pos_y);
  // some spontaneous duplication required now...!
  shared_ptr<const CarvingToolSpeed> tool_speed = settings->getToolSpeed(tool_name, CarvingModule::workpiece_material_name);
  double step = tool_speed->getStepDown();

  CarvingModule::current_z = step > thickness ? -thickness : -step;
  bool reverse = false;
  while (1) {
    CarvingSliceNode *slice_node = new CarvingSliceNode(inst, CarvingModule::current_z, reverse);
    node->children.push_back(slice_node);
    CarvingOperation *op = CarvingOperation::newGCodeOpPath2d(x, y, CarvingModule::current_z);
    slice_node->children.push_back(new CarvingOperationNode(inst, op, CarvingModule::current_z));

    std::vector<AbstractNode *> instantiatednodes = inst->instantiateChildren(evalctx);
    slice_node->children.insert(slice_node->children.end(), instantiatednodes.begin(), instantiatednodes.end());

    if (CarvingModule::current_z == -thickness) {
      break;
    }
    CarvingModule::current_z -= step;
    if (CarvingModule::current_z < -thickness) {
      CarvingModule::current_z = -thickness;
    }
    reverse = ! reverse;
  }
  PRINTDB("Carving: %s() end", inst->name());

  CarvingModule::current_z = nan("");    // not in path2d anymore

  return node;
}

AbstractNode *CarvingModule::instantiateLinearMoveNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  CarvingOperation *op = NULL;
  AssignmentList args;
  Context c(ctx);
  args += Assignment("v");
  c.setVariables(args, evalctx);

  if(!isfinite(CarvingModule::current_z)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_path2d() module", inst->name());
    return NULL;
  }

  double x = nan("");
  double y = nan("");
  if (ctx_get_vect2d(c, inst->name(), "v", x, y, true) < 0) {
    return NULL;
  }
  op = CarvingOperation::newGCodeOpLinearMoveXY(x, y);
  assert(op);
  PRINTDB("Carving: %s() instantiated %s, z %g", inst->name() % op->toString() % CarvingModule::current_z);
  return new CarvingOperationNode(inst, op, CarvingModule::current_z);
}

AbstractNode *CarvingModule::instantiateArcMoveNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  CarvingOperation *op = NULL;
  AssignmentList args;
  Context c(ctx);
  args += Assignment("v"), Assignment("center"), Assignment("ccw"), Assignment("p");
  c.setVariables(args, evalctx);

  if(!isfinite(CarvingModule::current_z)) {
    PRINTB("WARNING: Carving: %s() should be a child of carving_path2d() module", inst->name());
    return NULL;
  }

  double x = nan("");
  double y = nan("");
  if (ctx_get_vect2d(c, inst->name(), "v", x, y, true) < 0) {
    return NULL;
  }
  double i = nan("");
  double j = nan("");
  if (ctx_get_vect2d(c, inst->name(), "center", i, j, true) < 0) {
    return NULL;
  }
  const ValuePtr ccw_val = c.lookup_variable("ccw");
  bool ccw = ccw_val->type() == Value::BOOL ? ccw_val->toBool() : false;
  double p = ctx_get_double(c, "p", 1);

  if (x == i && y == j) {
    PRINTB("WARNING: Carving: %s(), arc end [%g, %g] and center [%g, %g] are identical, removing node.",
           inst->name() % x % y % i % j);
    return NULL;
  }

  op = CarvingOperation::newGCodeOpArcMoveXY(x, y, i, j, p, ccw);
  assert(op);
  return new CarvingOperationNode(inst, op, CarvingModule::current_z);
}

AbstractNode *CarvingModule::instantiateReverseNode(const Context *ctx, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  AssignmentList args;
  Context c(ctx);
  c.setVariables(args, evalctx);

  CarvingReverseNode *node = new CarvingReverseNode(inst);

  // add children
  std::vector<AbstractNode *> instantiatednodes = inst->instantiateChildren(evalctx);
  node->children.insert(node->children.end(), instantiatednodes.begin(), instantiatednodes.end());

  return node;
}


AbstractNode *CarvingModule::instantiateAssemblyNode(const Context */*ctx*/, const ModuleInstantiation *inst,
    EvalContext *evalctx) const
{
  CarvingAssemblyNode *node = new CarvingAssemblyNode(inst);
  // finally, take care of children
  std::vector<AbstractNode *> instantiatednodes = inst->instantiateChildren(evalctx);
  node->children.insert(node->children.end(), instantiatednodes.begin(), instantiatednodes.end());
  return node;
}

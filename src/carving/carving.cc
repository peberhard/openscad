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
#include "GeometryCache.h"
#include "CGALCache.h"
#include "ModuleCache.h"
#include "carving/carving-update-visitor.h"
#include "carving/carving-module.h"
#include "carving/carving-node.h"
#include "carving/gcode-export.h"
#include <fstream>


/* GCode class methods and class attributes */

Carving* Carving::_instance = NULL;

Carving* Carving::instance()
{
  if (!Carving::_instance) {
    Carving::_instance = new Carving();
  }
  return Carving::_instance;
}

/* GCode instance methods */

Carving::Carving()
  : rendering_mode_path(true), rendering_mode_result(false), rendering_mode_assembly(false), export_visitor(NULL)
{
}

Carving::~Carving()
{
}

void Carving::initialize() const
{
  Builtins::init(CARVING_SETTINGS_STR, new CarvingModule(CARVING_SETTINGS));
  Builtins::init(CARVING_TOOL_STR, new CarvingModule(CARVING_TOOL));
  Builtins::init(CARVING_TOOL_SPEED_STR, new CarvingModule(CARVING_TOOL_SPEED));
  Builtins::init(CARVING_WORKPIECE_STR, new CarvingModule(CARVING_WORKPIECE));
  Builtins::init(CARVING_DRILL_STR, new CarvingModule(CARVING_DRILL));
  Builtins::init(CARVING_PATH2D_STR, new CarvingModule(CARVING_PATH2D));
  Builtins::init(CARVING_LINEAR_MOVE_STR, new CarvingModule(CARVING_LINEAR_MOVE));
  Builtins::init(CARVING_ARC_MOVE_STR, new CarvingModule(CARVING_ARC_MOVE));
  Builtins::init(CARVING_REVERSE_STR, new CarvingModule(CARVING_REVERSE));
  Builtins::init(CARVING_ASSEMBLY_STR, new CarvingModule(CARVING_ASSEMBLY));
  Builtins::init(CARVING_PART_STR, new CarvingModule(CARVING_PART));
  Builtins::init(CARVING_SLICE_STR, new CarvingModule(CARVING_SLICE));
}

void Carving::updateCarvingContext(AbstractNode *root) const
{
  if (!root) {
    PRINT("WARNING: Carving: updateCarvingContext aborted, root node is null");
    return;
  }
  CarvingWorkpieceNode::clearPartTerms();
  GCodeExportVisitor::clearCache();
  if (this->export_visitor) {
    delete this->export_visitor;
    this->export_visitor = NULL;
  }

  CarvingUpdateVisitor update_cctx_visitor = CarvingUpdateVisitor(root);
  update_cctx_visitor.execute();
}

bool Carving::exportAsNGC(const AbstractNode *root, const char *file_name, const char *file_display_name) const
{
  if (!root) {
    PRINT("ERROR: Carving: Export to NGC failed, 'Preview' or 'Render' must be called first.");
    return false;
  }
  if (!file_name) {
    PRINT("ERROR: Carving: Export to NGC failed, filename is empty.");
    return false;
  }
  if (!file_display_name) {
    file_display_name = file_name;
  }
  PRINTDB("export '%s' begin...", file_display_name);

  std::ofstream fstream(file_name);
  if (!fstream.is_open()) {
    PRINTB("ERROR: Carving: Export to NGC failed, can't open file '%s'.", file_display_name);
    return false;
  }
  bool success = false;
  fstream.exceptions(std::ios::badbit | std::ios::failbit);
  try {
    GCodeExportVisitor::clearCache();
    this->export_visitor = new GCodeExportVisitor(root);
    this->export_visitor->execute();
    this->export_visitor->toNgc(fstream);
    success = true;
  } catch (std::ios::failure &x) {
  }
  try { // make sure file closed - resources released
    fstream.close();
  } catch (std::ios::failure &x) {
  }
  if (!success) {
    PRINTB("ERROR: Carving: Export to NGC failed, write error to '%s'.", file_display_name);
  }
  PRINTDB("export '%s' end.", file_display_name);
  return success;
}


shared_ptr<class CarvingSettings> Carving::getSettings()
{
  return this->settings;
}

void Carving::setSettings(shared_ptr<class CarvingSettings> settings)
{
  this->settings = settings;
}


static void flushCaches()
{
  GeometryCache::instance()->clear();
  CGALCache::instance()->clear();
  ModuleCache::instance()->clear();
}

bool Carving::isRenderingModePath() const
{
  return this->rendering_mode_path;
}

void Carving::setRenderingModePath()
{
  if (this->rendering_mode_path != true) {
    flushCaches();
  }
  this->rendering_mode_path = true;
  this->rendering_mode_result = false;
  this->rendering_mode_assembly = false;
}

bool Carving::isRenderingModeResult() const
{
  return this->rendering_mode_result;
}

void Carving::setRenderingModeResult()
{
  if (this->rendering_mode_result != true) {
    flushCaches();
  }
  this->rendering_mode_path = false;
  this->rendering_mode_result = true;
  this->rendering_mode_assembly = false;
}

bool Carving::isRenderingModeAssembly() const
{
  return this->rendering_mode_assembly;
}

void Carving::setRenderingModeAssembly()
{
  if (this->rendering_mode_assembly != true) {
    flushCaches();
  }
  this->rendering_mode_path = false;
  this->rendering_mode_result = false;
  this->rendering_mode_assembly = true;
}


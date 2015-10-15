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
#include "csgnode.h"
#include "linalg.h"
#include "csgterm.h"
#include "carving/carving-settings.h"
#include <sstream>
#include <map>


#define CARVING_SETTINGS_STR "carving_settings"
#define CARVING_TOOL_STR "carving_tool"
#define CARVING_TOOL_SPEED_STR "carving_tool_speed"

#define CARVING_WORKPIECE_STR "carving_workpiece"
#define CARVING_MATERIAL_STR "carving_material"
#define CARVING_DRILL_STR "carving_drill"
#define CARVING_PATH2D_STR "carving_path2d"
#define CARVING_RAPID_MOVE_STR "carving_rapid_move"
#define CARVING_LINEAR_MOVE_STR "carving_line"
#define CARVING_ARC_MOVE_STR "carving_arc"
#define CARVING_REVERSE_STR "carving_reverse"
#define CARVING_ASSEMBLY_STR "carving_assembly"
#define CARVING_PART_STR "carving_part"
#define CARVING_SLICE_STR "carving_slice"

enum carving_type_e
{
  CARVING_SETTINGS,     // general G-Code related parameters
  CARVING_TOOL,         // tool settings
  CARVING_TOOL_SPEED,   // tool speed depending on material
  CARVING_WORKPIECE,    // piece of material which will be milled
  CARVING_DRILL,        // simple drill
  CARVING_PATH2D,       // one path between 2 drill up
  CARVING_RAPID_MOVE,   // G0 rapid move http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G0
  CARVING_LINEAR_MOVE,  // G1 linear move http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G1
  CARVING_ARC_MOVE,     // G2 arc http://linuxcnc.org/docs/html/gcode/gcode.html#sec:G2-G3-Arc
  CARVING_REVERSE,      // reverse children operation order
  CARVING_ASSEMBLY,     // node gathering assembly parts and operations
  CARVING_PART,         // recall a part during assembly
  CARVING_SLICE,         // path2d slice
};



class CarvingPath2dNode;
class CarvingWorkpieceNode: public CsgNode
{
public:
  CarvingWorkpieceNode(const ModuleInstantiation *mi, const std::string &material_name, double x, double y,
      double thickness);
  virtual ~CarvingWorkpieceNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  const std::string & getMaterialName() const;
  double getX() const;
  double getY() const;
  double getThickness() const;

  static shared_ptr<CSGTerm> getPartTerm(const std::string &id);
  static void setPartTerm(const std::string &id, shared_ptr<CSGTerm> term);
  static void clearPartTerms();
  void addPath2dChildren(const CarvingPath2dNode *node) const;
private:
  CarvingWorkpieceNode();	// Declaration without definition (disallow default constructor)

  std::string material_name;
  double x, y;
  double thickness;
  mutable std::list<const CarvingPath2dNode*> path2d_children;
  // static
  static std::map<std::string, shared_ptr<CSGTerm> > part_terms;

};
std::ostream &operator<<(std::ostream &stream, const CarvingWorkpieceNode &o);



class CarvingPartNode: public CsgNode
{
public:
  CarvingPartNode(const ModuleInstantiation *mi, const std::string &id);
  virtual ~CarvingPartNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  const std::string & getId() const;

private:
  CarvingPartNode();   // Declaration without definition (disallow default constructor)
  std::string id;
};
std::ostream &operator<<(std::ostream &stream, const CarvingPartNode &o);


class CarvingMaterialNode: public LeafNode
{
public:
  CarvingMaterialNode(const ModuleInstantiation *mi, const std::string &material_name, double x, double y,
      double thickness);
  virtual ~CarvingMaterialNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  virtual class Geometry *createGeometry() const;
  const std::string& getMaterialName() const;
  double getLength() const;
  double getWidth() const;
  double getThickness() const;
private:
  CarvingMaterialNode();	// forbidden default constructor

  std::string material_name;
  double length, width;
  double thickness;
};
std::ostream &operator<<(std::ostream &stream, const CarvingMaterialNode &o);



class CarvingDrillNode: public LeafNode
{
public:
  CarvingDrillNode(const ModuleInstantiation *mi, const std::string &tool_name, double x, double y, double thickness);
  virtual ~CarvingDrillNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  virtual class Geometry *createGeometry() const;
  const std::string& getToolName() const;
  double getX() const;
  double getY() const;
  void setX(double x) const;
  void setY(double y) const;
  double getThickness() const;

private:
  CarvingDrillNode();	// forbidden default constructor

  std::string tool_name;
  mutable double x, y;
  double thickness;
};
std::ostream &operator<<(std::ostream &stream, const CarvingDrillNode &o);



class CarvingPath2dNode: public CsgNode
{
public:
  CarvingPath2dNode(const ModuleInstantiation *mi, const std::string &tool_name, double x, double y, double thickness,
      const std::string &id, double pos_x, double pos_y);
  virtual ~CarvingPath2dNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  const std::string& getToolName() const;
  double getX() const;
  void setX(double x);
  double getY() const;
  void setY(double y);
  double getThickness() const;
  const std::string& getId() const;
  double getPosX() const;
  void setPosX(double x);
  double getPosY() const;
  void setPosY(double y);
  const Transform3d& getPosMatrix() const;

private:
  CarvingPath2dNode();	// forbidden default constructor

  std::string tool_name;
  double x, y;
  double thickness;
  std::string id;
  double pos_x, pos_y;

  mutable Transform3d pos_matrix;
  mutable const class CarvingOperationNode *last_op;
};
std::ostream &operator<<(std::ostream &stream, const CarvingPath2dNode &o);


class CarvingSliceNode: public CsgNode
{
public:
  CarvingSliceNode(const ModuleInstantiation *mi, double z, bool reverse);
  virtual ~CarvingSliceNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  virtual double getZ() const;
  virtual bool getReverse() const;
private:
  CarvingSliceNode();  // forbidden default constructor
  double z;
  bool reverse;
};
std::ostream &operator<<(std::ostream &stream, const CarvingPath2dNode &o);


class CarvingOperationNode: public LeafNode
{
public:
	CarvingOperationNode(const ModuleInstantiation *mi, class CarvingOperation *op, double z);
	virtual ~CarvingOperationNode();
	virtual Response accept(class State &state, Visitor &visitor) const;
	virtual std::string toString() const;
	virtual std::string name() const;
	virtual class Geometry *createGeometry() const;
    void setOperationContext(shared_ptr<const CarvingTool> tool, shared_ptr<const CarvingToolSpeed> tool_speed,
        double thickness, const CarvingOperationNode *previous_op_node) const;
    virtual class CarvingOperation* getOp() const { return this->op; }
    void setOp(CarvingOperation *op) { this->op = op; }
    virtual const CarvingOperationNode* getPreviousOpNode() const { return this->previous_op_node; }
    virtual void setPreviousOpNode(const CarvingOperationNode* p) const { this->previous_op_node = p; }

private:
	CarvingOperationNode();	// forbidden default constructor
    void createGeometry(class PolySet *p, double last_x, double last_y, double current_z,
            const class CarvingOperation *op, const CarvingTool& tool) const;
    mutable shared_ptr<const CarvingTool> tool;
    mutable shared_ptr<const CarvingToolSpeed> tool_speed;
    double z;
    mutable double thickness;
    mutable CarvingOperation *op;
    mutable const CarvingOperationNode *previous_op_node;
};
std::ostream &operator<<(std::ostream &stream, const CarvingOperationNode &o);


class CarvingReverseNode: public CsgNode
{
public:
  CarvingReverseNode(const ModuleInstantiation *mi);
  virtual ~CarvingReverseNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
  mutable double next_x, next_y;        // TODO Carving: Hack to be cleaned...

private:
  CarvingReverseNode();  // forbidden default constructor
};
std::ostream &operator<<(std::ostream &stream, const CarvingReverseNode &o);


class CarvingAssemblyNode: public AbstractNode
{
public:
  CarvingAssemblyNode(const ModuleInstantiation *mi);
  virtual ~CarvingAssemblyNode();
  virtual Response accept(class State &state, class Visitor &visitor) const;
  virtual std::string name() const;
  virtual std::string toString() const;
private:
  CarvingAssemblyNode();   // Declaration without definition (disallow default constructor)
};
std::ostream &operator<<(std::ostream &stream, const CarvingAssemblyNode &o);

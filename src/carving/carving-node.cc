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

#include "printutils.h"
#include "cgalutils.h"
#include "state.h"
#include "polyset.h"
#include "cgalutils.h"
#include "csgterm.h"
#include "CSGTermEvaluator.h"
#include "GeometryEvaluator.h"
#include "carving/carving.h"
#include "carving/carving-node.h"
#include "carving/carving-operation.h"
#include "carving/carving-geometry.h"

#include <boost/math/special_functions/fpclassify.hpp>
#include <boost/foreach.hpp>
#include <QTime>

using boost::math::isfinite;
using Eigen::Vector3d;


/*
 * CarvingWorkpieceNode
 */

static const CarvingWorkpieceNode *current_workpiece = NULL;

std::map<std::string, shared_ptr<CSGTerm> > CarvingWorkpieceNode::part_terms;

shared_ptr<CSGTerm> CarvingWorkpieceNode::getPartTerm(const std::string &id)
{
  std::map<std::string, shared_ptr<CSGTerm> >::iterator iter;
  iter = CarvingWorkpieceNode::part_terms.find(id);
  if (iter == part_terms.end()) {
    return shared_ptr<CSGTerm>();
  }
  return iter->second;
}

void CarvingWorkpieceNode::setPartTerm(const std::string &id, shared_ptr<CSGTerm> term)
{
  assert(!id.empty());
  CarvingWorkpieceNode::part_terms[id] = term;
}

void CarvingWorkpieceNode::clearPartTerms()
{
  CarvingWorkpieceNode::part_terms.clear();
}

CarvingWorkpieceNode::CarvingWorkpieceNode(const ModuleInstantiation *mi, const std::string &material_name, double x,
    double y, double thickness) :
    CsgNode(mi, Carving::instance()->isRenderingModePath() ? OPENSCAD_UNION : OPENSCAD_DIFFERENCE),
    material_name(material_name), x(x), y(y), thickness(thickness)
{
  assert(!this->material_name.empty());
  assert(isfinite(this->x) && isfinite(this->y) && isfinite(this->thickness) && this->thickness > 0);
}

CarvingWorkpieceNode::~CarvingWorkpieceNode()
{
}

std::string CarvingWorkpieceNode::name() const
{
  return CARVING_WORKPIECE_STR;
}

std::string CarvingWorkpieceNode::toString() const
{
  std::stringstream stream;
  stream << this->name() << "(material=" << this->material_name << ", x=" << this->x << ", y=" << this->y
      << ", thickness=" << this->thickness << ")";
  return stream.str();
}

const std::string& CarvingWorkpieceNode::getMaterialName() const
{
  return this->material_name;
}

double CarvingWorkpieceNode::getX() const
{
  return this->x;
}

double CarvingWorkpieceNode::getY() const
{
  return this->y;
}

double CarvingWorkpieceNode::getThickness() const
{
  return this->thickness;
}

void CarvingWorkpieceNode::addPath2dChildren(const CarvingPath2dNode *node) const
{
  assert(node);
  this->path2d_children.push_back(node);
}

Response CarvingWorkpieceNode::accept(State &state, Visitor &visitor) const
{
  PRINTDB("CarvingWorkpieceNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  Response response = visitor.visit(state, *this);

  if (!Carving::instance()->isRenderingModeAssembly()) {
    return response;
  }
  if (typeid(visitor) != typeid(CSGTermEvaluator)) {
    return response;
  }
  if (state.isPrefix()) {
    assert(current_workpiece == NULL);
    assert(path2d_children.empty());
    current_workpiece = this;
    return response;
  }

  // figure out parts...
  QTime t;
  t.start();
  CSGTermEvaluator *csg = dynamic_cast<CSGTermEvaluator*>(&visitor);
  GeometryEvaluator geomevaluator(csg->tree);
  shared_ptr<const Geometry> root_geom = geomevaluator.evaluateGeometry(*this, true);
  PRINTDB("CarvingWorkpieceNode::accept geomevaluator.evaluateGeometry t %d", t.elapsed());
  t.restart();
  assert(root_geom && root_geom->getDimension() == 3);
  const CGAL_Nef_polyhedron *N = dynamic_cast<const CGAL_Nef_polyhedron*>(root_geom.get());
  assert(N);

  // for each part identified, set carving_node with the associated geometry
  shared_ptr<CSGTerm> new_csg;
  BOOST_FOREACH(const CarvingPath2dNode *ident_node, this->path2d_children) {
    if (ident_node->getId().empty()) {
      continue;
    }
    std::stringstream stream;
    stream << ident_node->name() << ident_node->index();
    shared_ptr<PolySet> ps(new PolySet(3));

    Vector3d v(ident_node->getPosX(), ident_node->getPosY(), 0);
    v = ident_node->getPosMatrix() * v;
    double pos_x = v.x();
    double pos_y = v.y();
    extract_volume_from_net_polyhedron(*N, pos_x, pos_y, 0, ps);
    new_csg.reset(new CSGTerm(ps, state.matrix(), state.color(), stream.str()));
    setPartTerm(ident_node->getId(), new_csg);

#if 0 /* FIXME Carving: Workspace must be evaluated before carving_part in mode="parts"  */
    // now check if part nodes where already trough. If yet, update them with geometry
    std::pair <part_type::iterator, part_type::iterator> range_iter;
    range_iter = parts.equal_range(ident_node->getId());
    for(part_type::iterator iter = range_iter.first; iter != range_iter.second; ++iter) {
      int index = iter->second;
      if (csg->stored_term.count(index) > 0)
        csg->stored_term[index]->geom = ps;
    }
#endif
  } // end foreach children
  csg->stored_term[this->index()].reset();
#if 0 // full render
  std::stringstream stream;
  stream << this->name() << this->index();
  shared_ptr<PolySet> ps(new PolySet(3));
  ps->setConvexity(N->getConvexity());
  bool err = CGALUtils::createPolySetFromNefPolyhedron3(*N->p3, *ps);
  assert(!err);
  csg->stored_term[this->index()].reset(new CSGTerm(ps , state.matrix(), state.color(), stream.str()));
#endif
  PRINTDB("CarvingWorkpieceNode::accept extract_volume_from_net_polyhedron t %d", t.elapsed());
  current_workpiece = NULL;
  this->path2d_children.clear();
  return response;
}

std::ostream &operator<<(std::ostream &stream, const CarvingWorkpieceNode &o)
{
  stream << o.toString();
  return stream;
}

/*
 * CarvingPartNode
 */

CarvingPartNode::CarvingPartNode(const ModuleInstantiation *mi, const std::string &id)
  :  CsgNode(mi, OPENSCAD_UNION), id(id)
{
  assert(!this->id.empty());
}

CarvingPartNode::~CarvingPartNode()
{
}

std::string CarvingPartNode::name() const
{
  return CARVING_PART_STR;
}

std::string CarvingPartNode::toString() const
{
  std::stringstream stream;
  stream << this->name() << "(id=" << this->id << ")";
  return stream.str();
}

const std::string& CarvingPartNode::getId() const
{
  return this->id;
}

Response CarvingPartNode::accept(State &state, Visitor &visitor) const
{
  PRINTDB("CarvingPartNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  Response response = visitor.visit(state, *this);
  if (state.isPostfix() && typeid(visitor) == typeid(CSGTermEvaluator)) {
    //PRINTDB("looking for %s", this->getId());
    shared_ptr<CSGTerm> id_term = CarvingWorkpieceNode::getPartTerm(this->getId());
    if (id_term) {
      PRINTDB("CSG stored_term for %s found", this->getId());
      std::stringstream stream;
      stream << this->name() << this->index();
      CSGTermEvaluator *csg = dynamic_cast<CSGTermEvaluator*>(&visitor);

      shared_ptr<const Geometry> g = id_term->geom;
      shared_ptr<CSGTerm> new_term(new CSGTerm(g, state.matrix(), state.color(), stream.str()));
      csg->stored_term[this->index()] = new_term;
    }
  }
  return response;
}

std::ostream &operator<<(std::ostream &stream, const CarvingPartNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingMaterialNode
 */

CarvingMaterialNode::CarvingMaterialNode(const ModuleInstantiation *mi, const std::string &material_name, double length,
    double width, double thickness) :
    LeafNode(mi), material_name(material_name), length(length), width(width), thickness(thickness)
{
  assert(!this->material_name.empty());
  assert(isfinite(this->length) && isfinite(this->width) && isfinite(this->thickness));
  assert(this->length > 0 && this->width > 0 && this->thickness > 0);
}

CarvingMaterialNode::~CarvingMaterialNode()
{
}

std::string CarvingMaterialNode::name() const
{
  return CARVING_MATERIAL_STR;
}

std::string CarvingMaterialNode::toString() const
{
  std::stringstream stream;
  stream << this->name() << "(material_name=" << this->material_name << ", length=" << this->length << ", width="
      << this->width << ", thickness=" << this->thickness << ")";
  return stream.str();
}

const std::string& CarvingMaterialNode::getMaterialName() const
{
  return this->material_name;
}

double CarvingMaterialNode::getLength() const
{
  return this->length;
}

double CarvingMaterialNode::getWidth() const
{
  return this->width;
}

double CarvingMaterialNode::getThickness() const
{
  return this->thickness;
}

Response CarvingMaterialNode::accept(State &state, Visitor &visitor) const
{
//	PRINTDB("CarvingMaterialNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  return visitor.visit(state, *this);
}

Geometry *CarvingMaterialNode::createGeometry() const
{
  PRINTDB("CarvingMaterialNode::createGeometry %s", *this);
  int rc;
  PolySet *p = new PolySet(3, true);

  double z = DEFAULT_Z - (this->getThickness() - 0.01);
  rc = generate_rectangle(p, 0, 0, z, this->getLength(), this->getWidth(), this->getThickness() - 2 * 0.01);
  if (rc) {
    PRINTB("WARNING: Carving: Failed to create carving_material geometry, generate_rectangle failed, node %s", *this);
  }
  return p;
}

std::ostream &operator<<(std::ostream &stream, const CarvingMaterialNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingDrillNode
 */

CarvingDrillNode::CarvingDrillNode(const ModuleInstantiation *mi, const std::string &tool_name, double x, double y,
    double thickness) :
    LeafNode(mi), tool_name(tool_name), x(x), y(y), thickness(thickness)
{
  assert(!this->tool_name.empty());
  assert(isfinite(this->x) && isfinite(this->y));
  assert(!isfinite(this->thickness) || this->thickness > 0);
}

CarvingDrillNode::~CarvingDrillNode()
{
}

std::string CarvingDrillNode::name() const
{
  return CARVING_DRILL_STR;
}

std::string CarvingDrillNode::toString() const
{
  // note: required in toString to generate proper stringId for geometry cache.
  std::stringstream stream;
  stream << this->name() << "(tool_name=" << this->tool_name << ", x=" << this->x << ", y=" << this->y << ", thickness="
      << this->thickness << ")";
  return stream.str();
}

const std::string& CarvingDrillNode::getToolName() const
{
  return this->tool_name;
}

double CarvingDrillNode::getX() const
{
  return this->x;
}

void CarvingDrillNode::setX(double x) const
{
  this-> x = x;
}

double CarvingDrillNode::getY() const
{
  return this->y;
}

void CarvingDrillNode::setY(double y) const
{
  this-> y = y;
}


double CarvingDrillNode::getThickness() const
{
  return this->thickness;
}

Geometry *CarvingDrillNode::createGeometry() const
{
  PRINTDB("CarvingDrillNode::createGeometry %s", *this);
  int rc;
  PolySet *p = new PolySet(3, true);
  assert(this->getChildren().empty());
  shared_ptr<const CarvingSettings> s = Carving::instance()->getSettings();
  shared_ptr<const CarvingTool> tool = s->getTool(this->getToolName());

  // tool.head_end is a size to be added to the drill depth in order to get though properly
  double headend = isfinite(tool->getHeadEnd()) ? tool->getHeadEnd() : 0;
  double height = this->getThickness() + headend;
  double z = DEFAULT_Z - height;
  rc = generate_cylinder(p, height, tool->getRadius(), this->getX(), this->getY(), z, s->getFn(), s->getFs(),
      s->getFa());
  if (rc) {
    PRINTB("WARNING: Carving: Failed to create carving_drill geometry, generate_cylinder failed, node %s", *this);
  }
  return p;
}

Response CarvingDrillNode::accept(State &state, Visitor &visitor) const
{
//	PRINTDB("CarvingDrillNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  return visitor.visit(state, *this);
}

std::ostream &operator<<(std::ostream &stream, const CarvingDrillNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingPath2dNode
 */

CarvingPath2dNode::CarvingPath2dNode(const ModuleInstantiation *mi, const std::string &tool_name, double x, double y,
    double thickness, const std::string &id, double pos_x, double pos_y) :
    CsgNode(mi, OPENSCAD_UNION), tool_name(tool_name), x(x), y(y), thickness(thickness),
    id(id), pos_x(pos_x), pos_y(pos_y), last_op(NULL)
{
  assert(!this->tool_name.empty());
  assert(isfinite(this->x) && isfinite(this->y));
  assert(!isfinite(this->thickness) || this->thickness > 0);
  assert(this->id.empty() || (isfinite(this->pos_x) && isfinite(this->pos_y)));
}

CarvingPath2dNode::~CarvingPath2dNode()
{
}

std::string CarvingPath2dNode::name() const
{
  return CARVING_PATH2D_STR;
}

std::string CarvingPath2dNode::toString() const
{
  // note: Required in toString to generate proper stringId for geometry cache.
  std::stringstream stream;
  stream << this->name() << "(tool_name=" << this->tool_name << ", x=" << this->x << ", y=" << this->y << ", thickness="
      << this->thickness << ")";
  return stream.str();
}

const std::string& CarvingPath2dNode::getToolName() const
{
  return this->tool_name;
}

double CarvingPath2dNode::getX() const
{
  return this->x;
}

void CarvingPath2dNode::setX(double x) {
  this->x = x;
}

double CarvingPath2dNode::getY() const
{
  return this->y;
}

void CarvingPath2dNode::setY(double y) {
  this->y = y;
}

double CarvingPath2dNode::getThickness() const
{
  return this->thickness;
}

const std::string& CarvingPath2dNode::getId() const
{
  return this->id;
}

double CarvingPath2dNode::getPosX() const
{
  return this->pos_x;
}

void CarvingPath2dNode::setPosX(double x) {
  this->pos_x = x;
}

double CarvingPath2dNode::getPosY() const
{
  return this->pos_y;
}

void CarvingPath2dNode::setPosY(double y) {
  this->pos_y = y;
}

const Transform3d &CarvingPath2dNode::getPosMatrix() const
{
  return this->pos_matrix;
}

Response CarvingPath2dNode::accept(State &state, Visitor &visitor) const
{
  //PRINTB("CarvingPath2dNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  if (state.isPrefix() && typeid(visitor) == typeid(CSGTermEvaluator)) {
    // index path2d for further identification later
    if (Carving::instance()->isRenderingModeAssembly()) {
      assert(current_workpiece);
      current_workpiece->addPath2dChildren(this);
      this->pos_matrix = state.matrix();
    }
  } // end if isPrefix && CSGTermEvaluator
  return visitor.visit(state, *this);
}

std::ostream &operator<<(std::ostream &stream, const CarvingPath2dNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvinSliceNode
 */

CarvingSliceNode::CarvingSliceNode(const ModuleInstantiation *mi, double z, bool reverse) :
    CsgNode(mi, OPENSCAD_UNION), z(z), reverse(reverse)
{
}

CarvingSliceNode::~CarvingSliceNode()
{
}

std::string CarvingSliceNode::name() const
{
  return CARVING_SLICE_STR;
}

std::string CarvingSliceNode::toString() const
{
  // note: Required in toString to generate proper stringId for geometry cache.
  std::stringstream stream;
  stream << this->name() << "(z=" << this->z << ") ";
  return stream.str();
}

Response CarvingSliceNode::accept(State &state, Visitor &visitor) const
{
//  PRINTB("CarvingSliceNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  return visitor.visit(state, *this);
}

double CarvingSliceNode::getZ() const
{
  return z;
}

bool CarvingSliceNode::getReverse() const
{
  return reverse;
}

std::ostream &operator<<(std::ostream &stream, const CarvingSliceNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingOperationNode
 */

CarvingOperationNode::CarvingOperationNode(const ModuleInstantiation *mi, CarvingOperation *op, double z) :
    LeafNode(mi), z(z), thickness(nan("")),  op(op), previous_op_node(NULL)

{
  assert(this->getOp() != NULL);
}

CarvingOperationNode::~CarvingOperationNode()
{
}

std::string CarvingOperationNode::name() const
{
  return "CarvingOperationNode";
}

void CarvingOperationNode::setOperationContext(shared_ptr<const CarvingTool> tool,
    shared_ptr<const CarvingToolSpeed> tool_speed, double thickness, const CarvingOperationNode *previous_op_node) const
{
  this->tool = tool;
  this->tool_speed = tool_speed;
  this->thickness = thickness;
  this->setPreviousOpNode(previous_op_node);
}

std::string CarvingOperationNode::toString() const
{
  // note: required in toString to generate proper stringId for geometry cache.
  std::stringstream stream;
  stream << this->name() << "(" << *this->getOp() << ",z " << this->z << ", previous_op ";
  if (this->getPreviousOpNode()) {
    stream << this->getPreviousOpNode()->getOp()->toString() << ")";
  } else {
    stream << "null)";
  }
  return stream.str();
}

Response CarvingOperationNode::accept(State &state, Visitor &visitor) const
{
  // PRINTDB("CarvingOperationNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  return visitor.visit(state, *this);
}

Geometry *CarvingOperationNode::createGeometry() const
{
  // PRINTDB("CarvingOperationNode::createGeometry %s", *this);

  int rc;
  PolySet *p = NULL;
  shared_ptr<const CarvingSettings> s = Carving::instance()->getSettings();

  if (this->getOp()->getType() == CARVING_OP_PATH2D) {
    return new PolySet(3, true);
  }

  double headend = isfinite(this->tool->getHeadEnd()) ? tool->getHeadEnd() : 0;
  double height = this->tool_speed->getStepDown();
  double r = tool->getRadius();


  if (!getPreviousOpNode()) {
    return new PolySet(3, true);
  }

  switch (this->getOp()->getType()) {
  case CARVING_OP_LINEAR_MOVE_XY:
    p = new PolySet(3, true);
    rc = generate_rectangle(p, getPreviousOpNode()->getOp()->getX(), getPreviousOpNode()->getOp()->getY(),
        this->z - headend, this->getOp()->getX(), this->getOp()->getY(), this->z + height, r, s->getFn(), s->getFs(),
        s->getFa());
    if (rc) {
      PRINTB("WARNING: Carving: failed to created carving_line() geometry, generate_rectangle failed, node %s", *this);
    }
    break;
  case CARVING_OP_ARC_MOVE_XY:
    p = new PolySet(3, false);
    rc = generate_arc(p, this->getOp()->getI(), this->getOp()->getJ(), this->z - headend,
        getPreviousOpNode()->getOp()->getX(), getPreviousOpNode()->getOp()->getY(), this->getOp()->getX(),
        this->getOp()->getY(), this->getOp()->getCcw(), height, r, s->getFn(), s->getFs(), s->getFa());
    if (rc) {
      PRINTB("WARNING: Carving: failed to create carving_arc() geometry, generate_arc failed, node %s", *this);
    }
    break;
  default:
    PRINTB("WARNING: Carving: failed to create geometry, unsupported node type, node '%s'.", *this);
  }
  return p ? p : new PolySet(3, true);
}

std::ostream &operator<<(std::ostream &stream, const CarvingOperationNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingReverseNode
 */

CarvingReverseNode::CarvingReverseNode(const ModuleInstantiation *mi)
    : CsgNode(mi, OPENSCAD_UNION), next_x(nan("")), next_y(nan(""))

{
}

CarvingReverseNode::~CarvingReverseNode()
{
}

std::string CarvingReverseNode::name() const
{
  return CARVING_REVERSE_STR;
}

std::string CarvingReverseNode::toString() const
{
  std::stringstream stream;
  stream << this->name() << "()";
  return stream.str();
}

Response CarvingReverseNode::accept(State &state, Visitor &visitor) const
{
  //PRINTDB("CarvingReverseNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());

  // TODO Carving implement reverse
  //
  return visitor.visit(state, *this);
}


std::ostream &operator<<(std::ostream &stream, const CarvingReverseNode &o)
{
  stream << o.toString();
  return stream;
}


/*
 * CarvingAssemblyNode
 */

CarvingAssemblyNode::CarvingAssemblyNode(const ModuleInstantiation *mi) : AbstractNode(mi)
{
}

CarvingAssemblyNode::~CarvingAssemblyNode()
{
}

std::string CarvingAssemblyNode::name() const
{
  return CARVING_ASSEMBLY_STR;
}

std::string CarvingAssemblyNode::toString() const
{
  std::stringstream stream;
  stream << this->name() << "()";
  return stream.str();
}

Response CarvingAssemblyNode::accept(State &state, Visitor &visitor) const
{
//  PRINTDB("CarvingAssemblyNode::accept visitor class %s prefix %d", typeid(visitor).name() % state.isPrefix());
  if (!Carving::instance()->isRenderingModeAssembly()) {
    return ContinueTraversal;
  }
  return visitor.visit(state, *this);
}

std::ostream &operator<<(std::ostream &stream, const CarvingAssemblyNode &o)
{
  stream << o.toString();
  return stream;
}


#include "printutils.h"
#include "carving/carving-operation.h"
#include "carving/carving-geometry.h"
#include <boost/math/special_functions/fpclassify.hpp>

using boost::math::isfinite;


/*
 * class methods
 */

CarvingOperation *CarvingOperation::newGCodeOpPath2d(double x, double y, double z)
{
  return new CarvingOperation(CARVING_OP_PATH2D, x, y, z);
}

CarvingOperation *CarvingOperation::newGCodeOpRapidMoveXY(double x, double y)
{
  return new CarvingOperation(CARVING_OP_RAPID_MOVE_XY, x, y);
}

CarvingOperation *CarvingOperation::newGCodeOpRapidMoveZ(double z)
{
  return new CarvingOperation(CARVING_OP_RAPID_MOVE_Z, z);
}

CarvingOperation *CarvingOperation::newGCodeOpLinearMoveXY(double x, double y)
{
  return new CarvingOperation(CARVING_OP_LINEAR_MOVE_XY, x, y);
}

CarvingOperation *CarvingOperation::newGCodeOpLinearMoveZ(double z)
{
  return new CarvingOperation(CARVING_OP_LINEAR_MOVE_Z, z);
}

CarvingOperation *CarvingOperation::newGCodeOpArcMoveXY(double x, double y, double i, double j, double p, bool ccw)
{
  return new CarvingOperation(CARVING_OP_ARC_MOVE_XY, x, y, i, j, p, ccw);
}

/*
 * Instance methods
 */

std::string CarvingOperation::toString() const
{
  std::stringstream stream;
  if (this->type == CARVING_OP_PATH2D) {
    stream << this->name() << "(t=" << this->type << ", z=" << this->z << ", x=" << this->x << ", y=" << this->y << ")";
  } else if (this->type == CARVING_OP_RAPID_MOVE_XY || this->type == CARVING_OP_LINEAR_MOVE_XY) {
    stream << this->name() << "(t=" << this->type << ", x=" << this->x << ", y=" << this->y << ")";
  } else if (this->type == CARVING_OP_RAPID_MOVE_Z || this->type == CARVING_OP_LINEAR_MOVE_Z) {
    stream << this->name() << "(t=" << this->type << ", z=" << this->z << ")";
  } else if (this->type == CARVING_OP_ARC_MOVE_XY || this->type == CARVING_OP_ARC_MOVE_XY) {
    stream << this->name() << "(t=" << this->type << ", x=" << this->x << ", y=" << this->y << ", i=" << this->i
        << ", j=" << this->j << ", p=" << this->p << ", ccw=" << this->ccw << ")";
  } else {
    stream << this->name() << "(t=" << this->type << ")";
  }
  return stream.str();
}

std::string CarvingOperation::name() const
{
  switch (this->type) {
  case CARVING_OP_PATH2D:
    return "path2d";
  case CARVING_OP_RAPID_MOVE_XY:
    return "rapid_move_xy";
  case CARVING_OP_RAPID_MOVE_Z:
    return "rapid_move_z";
  case CARVING_OP_LINEAR_MOVE_XY:
    return "linear_move_xy";
  case CARVING_OP_LINEAR_MOVE_Z:
    return "linear_move_z";
  case CARVING_OP_ARC_MOVE_XY:
    return "arc_move_xy";
  default:
    return "unknown GCodeOp type";
  }
}

int CarvingOperation::getType() const
{
  return this->type;
}

double CarvingOperation::getX() const
{
  assert(isfinite(this->x));
  return this->x;
}

double CarvingOperation::getY() const
{
  assert(isfinite(this->y));
  return this->y;
}

double CarvingOperation::getZ() const
{
  assert(isfinite(this->z));
  return this->z;
}

double CarvingOperation::getI() const
{
  assert(isfinite(this->i));
  return this->i;
} // X offset

double CarvingOperation::getJ() const
{
  assert(isfinite(this->j));
  return this->j;
} // Y offset

double CarvingOperation::getP() const
{
  assert(isfinite(this->p));
  return this->p;
} // number of turns

bool CarvingOperation::getCcw() const
{
  return this->ccw;
} // counter-clockwise

void CarvingOperation::reverseCcw() const
{
  this->ccw = !this->ccw;
}

CarvingOperation *CarvingOperation::reverseTo(double to_x, double to_y) const // TODO Carving Obsolete
{
  CarvingOperation *rev_op = NULL;
  switch (this->getType()) {
  case CARVING_OP_LINEAR_MOVE_XY:
    rev_op = CarvingOperation::newGCodeOpLinearMoveXY(to_x, to_y);
    break;
  case CARVING_OP_ARC_MOVE_XY:
    rev_op = CarvingOperation::newGCodeOpArcMoveXY(to_x, to_y, this->i, this->j, this->p, !this->ccw);
    break;
  default:
    PRINTB("ERROR: Carving: reverseTo() failed, type not supported op %s x=%g, y=%g", *this % to_x % to_y);
  }
  return rev_op;
}

void CarvingOperation::transform(const Transform2d &matrix2d) const
{
  // FIXME Carving: handle ccw here
  if (type == CARVING_OP_LINEAR_MOVE_XY || type == CARVING_OP_ARC_MOVE_XY || type == CARVING_OP_PATH2D) {
    assert(isfinite(this->x) && isfinite(this->y));
    Eigen::Vector2d v(this->x, this->y);
    v = matrix2d * v;
    if (type == CARVING_OP_ARC_MOVE_XY) {
      assert(isfinite(this->i) && isfinite(this->j));
      Eigen::Vector2d v2(this->i, this->j);
      v2 = matrix2d * v2;
//      PRINTB("op tranform2d arc [%g, %g] c[%g, %g] => [%g, %g] c[%g, %g]", this->x % this->y % this->i % this->j %
//          v.x() % v.y() % v2.x() % v2.y());
      this->x = v.x();
      this->y = v.y();
      this->i = v2.x();
      this->j = v2.y();
      assert(isfinite(this->x) && isfinite(this->y));
      assert(isfinite(this->i) && isfinite(this->j));
      assert(!(this->x == this->i && this->y == this->j));
      if (is_matrix_reversing_angle(matrix2d)) {
        this->ccw = !this->ccw;
      }
    } else {
//      PRINTB("op tranform2d line [%g, %g] => [%g, %g]", this->x % this->y % v.x() % v.y());
      this->x = v.x();
      this->y = v.y();
      assert(isfinite(this->x) && isfinite(this->y));
    }
  } else {
    PRINTB("ERROR CarvingOperation::transform type not supported (%s)", *this);
    assert(0);
  }
}


/*
 * private methods
 */

CarvingOperation::CarvingOperation(carving_op_type_e type, double x, double y, double z) :
    type(type), x(x), y(y), z(z), i(nan("")), j(nan("")), p(nan("")), ccw(false)
{
  assert(type == CARVING_OP_PATH2D);
  assert(isfinite(this->z));
  assert(isfinite(this->x) && isfinite(this->y));
}

CarvingOperation::CarvingOperation(carving_op_type_e type, double x, double y) :
    type(type), x(x), y(y), z(nan("")), i(nan("")), j(nan("")), p(nan("")), ccw(false)
{
  assert(isfinite(this->x) && isfinite(this->y));
}

CarvingOperation::CarvingOperation(carving_op_type_e type, double z) :
    type(type), x(nan("")), y(nan("")), z(z), i(nan("")), j(nan("")), p(nan("")), ccw(false)
{
  assert(isfinite(this->z));
}

CarvingOperation::CarvingOperation(carving_op_type_e type, double x, double y, double i, double j, double p, bool ccw) :
    type(type), x(x), y(y), z(nan("")), i(i), j(j), p(p), ccw(ccw)
{
  assert(isfinite(this->x) && isfinite(this->y));
  assert(isfinite(this->i) && isfinite(this->j));
  assert(!(x == i && y == j));
}

/* friends */

std::ostream &operator<<(std::ostream &stream, const CarvingOperation &o)
{
  stream << o.toString();
  return stream;
}

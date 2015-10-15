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
#include "carving/gcode.h"
#include "carving/carving-geometry.h"
#include <math.h>
#include <typeinfo>
#include <boost/foreach.hpp>
#include <sstream>

using boost::dynamic_pointer_cast;

std::string GCode::toString() const {
  return this->toStr();
}

void GCode::toNgc(std::ostream &stream) const
{
//  PRINTB("GCode::toNgc '%s' %s", typeid(*this).name() % this->toStr());
  stream << this->toStr() << std::endl;
  if (this->children.size() != 0) {
    BOOST_FOREACH(shared_ptr<GCode> gc_ptr, this->children) {
      gc_ptr->toNgc(stream);
    }
  }
}

shared_ptr<GCode> GCode::cloneZ() const
{
  shared_ptr<GCode> new_gc;
  if (dynamic_cast<const GContainer*>(this)) {
    new_gc = make_shared<GContainer>(this->comment);
    BOOST_FOREACH(shared_ptr<GCode> child_ptr, this->children) {
      shared_ptr<GCode> new_child = child_ptr->cloneZ();
      new_gc->children.push_back(new_child);
    }
  } else {
    new_gc = this->clone();
  }
  return new_gc;
}


/*static*/ shared_ptr<GCode> GCode::lastMove(shared_ptr<GCode> gc)
{
  //PRINTDB("DEBUG last_move %s", *gc);
  if (dynamic_pointer_cast<G1>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<G2>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<G3>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<GContainer>(gc)) {
    shared_ptr<GCode> last;
    BOOST_FOREACH(shared_ptr<GCode> child_ptr, gc->children) {
      shared_ptr<GCode> last_tmp = GCode::lastMove(child_ptr);
      if (last_tmp) {
          last = last_tmp;
      }
    }
    return last;
  } else {
    //PRINTB("WARNING: Carving: GCode::lastMove() of this node does nothing, node '%s'", *gc);
    return shared_ptr<GCode>();
  }
}


/*static*/ shared_ptr<GCode> GCode::firstMove(shared_ptr<GCode> gc)
{
  //PRINTDB("DEBUG last_move %s", *gc);
  if (dynamic_pointer_cast<G1>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<G2>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<G3>(gc)) {
    return gc;
  } else if (dynamic_pointer_cast<GContainer>(gc)) {
    BOOST_FOREACH(shared_ptr<GCode> child_ptr, gc->children) {
      shared_ptr<GCode> first = GCode::firstMove(child_ptr);
      if (first) {
          return first;
      }
    }
  } else {
    //PRINTB("WARNING: Carving: GCode::lastMove() of this node does nothing, node '%s'", *gc);
  }
  return shared_ptr<GCode>();
}


std::ostream &operator<<(std::ostream &stream, const GCode &gc)
{
  stream << gc.toStr();
  return stream;
}

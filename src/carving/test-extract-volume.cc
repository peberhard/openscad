// g++ test-extract-volume.cc -frounding-math -lCGAL -lgmp -o test -W -Wall -I.. && ./test
//
#include <stdio.h>

#define ENABLE_CGAL
#include "cgal.h"
#include <iostream>
/*
 #include <CGAL/Simple_cartesian.h>
 #include <CGAL/Polyhedron_3.h>
 typedef CGAL::Simple_cartesian<double>     Kernel;
 typedef Kernel::Point_3                    Point_3;
 typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
 typedef Polyhedron::Vertex_iterator        Vertex_iterator;
 typedef Polyhedron::Halfedge_iterator      Halfedge_iterator;
 */
typedef CGAL_Polyhedron::Vertex_iterator Vertex_iterator;
typedef CGAL_Polyhedron::Halfedge_iterator Halfedge_iterator;

struct halfedge_compare
{
  bool operator()(const Halfedge_iterator& lhs, const Halfedge_iterator& rhs) const
  {
    std::stringstream s1, s2;
    s1 << lhs->vertex()->point() << lhs->next()->vertex()->point();
    s2 << rhs->vertex()->point() << rhs->next()->vertex()->point();
    std::cout << s1.str() << "   " << s2.str() << std::endl;
    return strcmp(s1.str().c_str(), s2.str().c_str());
  }
};

std::string key(const Halfedge_iterator &lhs)
{
  std::stringstream s1, s2;
  CGAL_Point_3 pt = lhs->vertex()->point();
  s1 << "[" << pt.x().to_double() << ", " << pt.y().to_double() << ", " << pt.z().to_double() << "] ";
  //s1 << lhs->vertex()->point() << lhs->next()->vertex()->point();
  return s1.str();
}

int main()
{
  CGAL_Point_3 p(1.0, 0.0, 0.0);
  CGAL_Point_3 q(0.0, 1.0, 0.0);
  CGAL_Point_3 r(0.0, 0.0, 1.0);
  CGAL_Point_3 s(0.0, 0.0, 0.0);
  CGAL_Polyhedron P;
  P.make_tetrahedron(p, q, r, s);
  CGAL::set_ascii_mode(std::cout);
  printf("vetices\n");
  for (Vertex_iterator v = P.vertices_begin(); v != P.vertices_end(); ++v)
    std::cout << v->point() << std::endl;
  printf("halfedges\n");
  for (Halfedge_iterator h = P.halfedges_begin(); h != P.halfedges_end(); ++h)
    std::cout << h->vertex()->point() << std::endl;

  std::set < std::string > already_saw;
  std::list<Halfedge_iterator> queue;
  queue.push_back(P.halfedges_begin());
  already_saw.insert(key(P.halfedges_begin()));
  while (!queue.empty()) {
    Halfedge_iterator begin = queue.front();
    Halfedge_iterator it = begin;
    queue.pop_front();
    std::cout << "halfedge face ";
    do {
      std::string k = key(it);
      if (already_saw.count(k) == 0) {
//        printf("                  adding halfedge %s\n", k.c_str());
        queue.push_back(it->opposite());
        already_saw.insert(k);
      }
      CGAL_Point_3 pt = it->vertex()->point();
      std::cout << "[" << pt.x().to_double() << ", " << pt.y().to_double() << ", " << pt.z().to_double() << "] ";

      it = it->next();
    } while (it->vertex()->point() != begin->vertex()->point());
    std::cout << std::endl;

  }

  return 0;
}
/*
 #define ENABLE_CGAL
 #include "cgal.h"

 #include <CGAL/Simple_cartesian.h>
 #include <CGAL/HalfedgeDS_vector.h>
 #include <CGAL/Polyhedron_3.h>

 typedef CGAL_Polyhedron::Halfedge_handle        Halfedge_handle;
 typedef CGAL_Kernel3::Point_3 Point_3;
 int main()
 {
 CGAL_Polyhedron P;
 Halfedge_handle face_start = P.make_tetrahedron();

 CGAL::set_ascii_mode( std::cout);
 for ( Vertex_iterator v = P.vertices_begin(); v != P.vertices_end(); ++v)
 std::cout << v->point() << std::endl;

 //http://doc.cgal.org/latest/Polyhedron/index.html#title14
 Halfedge_handle it = face_start;
 do {
 it->next();
 printf("ee\n");
 //      printf("%g\n", it->vertex()->point().x()->);
 } while (it != face_start);


 return 0;
 }

 */

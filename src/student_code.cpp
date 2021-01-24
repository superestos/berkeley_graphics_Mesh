#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{
  void BezierCurve::evaluateStep()
  {
    // TODO Part 1.
    // Perform one step of the Bezier curve's evaluation at t using de Casteljau's algorithm for subdivision.
    // Store all of the intermediate control points into the 2D vector evaluatedLevels.

    size_t level = evaluatedLevels.size();
    evaluatedLevels.push_back({});
    for (size_t i = 0; i < numControlPoints - level; i++) {
      evaluatedLevels[level].push_back(t * evaluatedLevels[level - 1][i] + (1 - t) * evaluatedLevels[level - 1][i + 1]);
    }

    return;
  }


  Vector3D BezierPatch::evaluate(double u, double v) const
  {
    // TODO Part 2.
    // Evaluate the Bezier surface at parameters (u, v) through 2D de Casteljau subdivision.
    // (i.e. Unlike Part 1 where we performed one subdivision level per call to evaluateStep, this function
    // should apply de Casteljau's algorithm until it computes the final, evaluated point on the surface)

    size_t n = controlPoints.size();
    std::vector<Vector3D> vPoints;

    for (size_t i = 0; i < n; i++) {
      vPoints.push_back(evaluate1D(controlPoints[i], u));
    }

    return evaluate1D(vPoints, v);
  }

  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> points, double t) const
  {
    // TODO Part 2.
    // Optional helper function that you might find useful to implement as an abstraction when implementing BezierPatch::evaluate.
    // Given an array of 4 points that lie on a single curve, evaluates the Bezier curve at parameter t using 1D de Casteljau subdivision.

    size_t n = points.size();
    for (size_t i = 1; i < n; i++) {
      for (size_t j = 0; j < n - i; j++) {
        points[j] = points[j] * t + points[j + 1] * (1 - t);
      }
    }

    return points[0];
 }



  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.

    Vector3D N;
    auto h = halfedge();

    do {
      N += h->face()->normal();
      h = h->twin()->next();
    } while (h != halfedge());

    return N.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // TODO This method should flip the given edge and return an iterator to the flipped edge.

    if (e0->isBoundary()) {
      return e0;
    }

    auto h0 = e0->halfedge();
    auto h1 = h0->next();
    auto h2 = h1->next();
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = h4->next();

    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto v2 = h2->vertex();
    auto v3 = h5->vertex();

    auto f0 = h0->face();
    auto f1 = h3->face();

    h0->setNeighbors(h2, h3, v3, e0, f0);
    h3->setNeighbors(h5, h0, v2, e0, f1);
    h1->setNeighbors(h3, h1->twin(), h1->vertex(), h1->edge(), f1);
    h2->setNeighbors(h4, h2->twin(), h2->vertex(), h2->edge(), f0);
    h4->setNeighbors(h0, h4->twin(), h4->vertex(), h4->edge(), f0);
    h5->setNeighbors(h1, h5->twin(), h5->vertex(), h5->edge(), f1);

    v0->halfedge() = h4;
    v1->halfedge() = h1;
    v2->halfedge() = h2;
    v3->halfedge() = h5;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
    // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    
    if (e0->isBoundary()) {
      return e0->halfedge()->vertex();
    }

    auto h0 = e0->halfedge();
    auto h1 = h0->next();
    auto h2 = h1->next();
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = h4->next();

    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto v2 = h2->vertex();
    auto v3 = h5->vertex();

    auto f0 = h0->face();
    auto f1 = h3->face();

    auto v4 = newVertex();
    auto e1 = newEdge();
    auto e2 = newEdge();
    auto e3 = newEdge();
    auto f2 = newFace();
    auto f3 = newFace();

    auto h6 = newHalfedge();
    auto h7 = newHalfedge();
    auto h8 = newHalfedge();
    auto h9 = newHalfedge();
    auto h10 = newHalfedge();
    auto h11 = newHalfedge();

    h0->setNeighbors(h7, h9, v0, e0, f0);
    h1->setNeighbors(h8, h1->twin(), h1->vertex(), h1->edge(), f2);
    h2->setNeighbors(h0, h2->twin(), h2->vertex(), h2->edge(), f0);
    h3->setNeighbors(h10, h6, v1, e1, f1);
    h4->setNeighbors(h11, h4->twin(), h4->vertex(), h4->edge(), f3);
    h5->setNeighbors(h3, h5->twin(), h5->vertex(), h5->edge(), f1);
    h6->setNeighbors(h1, h3, v4, e1, f2);
    h7->setNeighbors(h2, h8, v4, e2, f0);
    h8->setNeighbors(h6, h7, v2, e2, f2);
    h9->setNeighbors(h4, h0, v4, e0, f3);
    h10->setNeighbors(h5, h11, v4, e3, f1);
    h11->setNeighbors(h9, h10, v3, e3, f3);

    v4->halfedge() = h6;

    e1->halfedge() = h6;
    e2->halfedge() = h7;
    e3->halfedge() = h10;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f2->halfedge() = h1;
    f3->halfedge() = h4;

    v4->position = (v0->position + v1->position) * 0.5;

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
    // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
    // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
    // the new subdivided (fine) mesh, which has more elements to traverse. We will then assign vertex positions in
    // the new mesh based on the values we computed for the original mesh.


    // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // TODO a vertex of the original mesh.
    for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
      double u = v->degree() == 3? 3.0 / 16.0: 3.0 / (8.0 * (double)v->degree());
      v->newPosition = (1 - (double)v->degree() * u) * v->position;
      v->isNew = false;
      
      auto h = v->halfedge();
      do {
        v->newPosition += u * h->next()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());
    }    

    // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
      auto h = e->halfedge();
      e->newPosition = 3.0 / 8.0 * (h->vertex()->position + h->twin()->vertex()->position);
      e->newPosition += 1.0 / 8.0 * (h->next()->next()->vertex()->position + h->twin()->next()->next()->vertex()->position);
      e->isNew = false;
    }

    // TODO Next, we're going to split every edge in the mesh, in any order.  For future
    // TODO reference, we're also going to store some information about which subdivided
    // TODO edges come from splitting an edge in the original mesh, and which edges are new,
    // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
    // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
    // TODO just split (and the loop will never end!)
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
      if (!e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew && !e->isBoundary()) {
        auto h1 = e->halfedge();
        auto h2 = e->halfedge()->twin();

        auto mid = mesh.splitEdge(e);
        mid->newPosition = e->newPosition;
        mid->isNew = true;

        h1->next()->edge()->isNew = true;
        h2->next()->edge()->isNew = true;
      }
    }

    // TODO Now flip any new edge that connects an old and new vertex.
    for (auto h = mesh.halfedgesBegin(); h != mesh.halfedgesEnd(); h++) {
      if (!h->vertex()->isNew && h->next()->vertex()->isNew && h->edge()->isNew) {
        h->edge()->isNew = false;
        mesh.flipEdge(h->edge());
      }
    }
    
    // TODO Finally, copy the new vertex positions into final Vertex::position.
    for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;
      v->position = v->newPosition;
    }

    return;
  }
}

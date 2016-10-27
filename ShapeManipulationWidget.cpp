#include "ShapeManipulationWidget.hpp"

#include <Luminous/Image.hpp>

#include "eigen/Eigen/Dense"
#include "eigen/Eigen/IterativeLinearSolvers"
#include "eigen/Eigen/SparseQR"
#include "eigen/Eigen/OrderingMethods"
#include "eigen/Eigen/StdVector"
#include "eigen/Eigen/SparseLU"
#include "eigen/Eigen/SparseCholesky"

#include <array>

#define ITERATIVE_SOLVE 1

typedef float RealType;

typedef Nimble::Vector2 TextureCoordinate;

// image size
const Nimble::Vector2 s_scale = 0.1f*Nimble::Vector2(5233, 6509);
// seems to somehow rotate funkily towards origin
const Nimble::Vector2 s_offset = -0.5f*s_scale;

// from [0,1]^2 to triangulation
const Nimble::Matrix3 toTriangulation = Nimble::Matrix3::makeTranslation(s_offset) *
    Nimble::Matrix3::makeScale(s_scale);
const Nimble::Matrix3 fromTriangulation = toTriangulation.inverse();

template<typename Payload>
struct Vertex
{
  Nimble::Vector2 location;
  Payload payload;

  //std::vector<int> neighbourIndices;
};

struct Edge
{
  int fromIndex, toIndex;
  std::vector<int> neighbourVertices;
};

struct Triangle
{
  int edges[3];

  int indexForEdgeIndex(int edgeIndex)
  {
    for(int i=0; i < 3; ++i) {
      if(edges[i] == edgeIndex)
        return i;
    }
    return -1;
  }

};


template<typename VertexPayload>
struct Triangulation
{
  // Edge-based triangulation
  std::vector<Vertex<VertexPayload>> m_vertices;
  std::vector<Edge> m_edges;
  std::vector<Triangle> m_triangles;
};

// As-Rigid-As-Possible shape
template<typename VertexPayload>
class ArapShape
{
public:
  // Divide [0,1] x [0,1] into triangles
  bool constructFromUnitRect(int xdivide, int ydivide)
  {
    float xinc = 1.0f/xdivide;
    float yinc = 1.0f/ydivide;
    int index = 0;

    Triangulation<VertexPayload> mesh;
    for(int y=0; y <= ydivide; ++y) {
      for(int x=0; x <= xdivide+1; ++x){
         // location is same as texture coordinate
         Nimble::Vector2 location(x*xinc, y*yinc);
         // offset every second row for hexagonal pattern
         if(y % 2 != 0) {
           location.x -= 0.5f*xinc;
         } else if(x == xdivide+1) {
           continue;
         }

         Nimble::Vector2 payload = location;
         location = toTriangulation.project(location);
#if 0
         std::vector<int> indices;
         if(x > 0) {
           indices.push_back(index-1);
         }
         if(y > 0) {
           indices.push_back(index-(xdivide+1));
         }
         if(x < xdivide) {
           indices.push_back(index+1);
           mesh.m_edges.push_back({index, index+1});
         }
         if(y < ydivide) {
           indices.push_back(index+(xdivide+1));
           mesh.m_edges.push_back({index, index+(xdivide+1)});
         }

         if(x < xdivide && y < ydivide) {
           mesh.m_edges.push_back({index+1, index+(xdivide+1)});
         }
         mesh.m_vertices.push_back(Vertex<VertexPayload> { location, location, indices } );
#else
         if(x < xdivide && y < ydivide) {
           if(y % 2 == 0) {
             mesh.m_edges.push_back({index, index+(xdivide+1)});
             mesh.m_edges.push_back({index+(xdivide+1), index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index});

             mesh.m_edges.push_back({index, index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index+1});
             mesh.m_edges.push_back({index+1, index});
           } else {
             mesh.m_edges.push_back({index, index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index+1});
             mesh.m_edges.push_back({index+1, index});

               /*
             mesh.m_edges.push_back({index, index+1});
             mesh.m_edges.push_back({index+1, index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index});
             */

             mesh.m_edges.push_back({index+1, index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index+(xdivide+1)+1+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1+1, index+1});
/*
             mesh.m_edges.push_back({index+(xdivide+1)+1, index+1});
             mesh.m_edges.push_back({index+1, index+(xdivide+1)+1+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1+1, index+(xdivide+1)+1});
             */
           }

           int e = mesh.m_edges.size()-1;
           mesh.m_triangles.push_back({e-5, e-4, e-3});
           mesh.m_triangles.push_back({e-2, e-1, e-0});
         } else if(x == xdivide && y < ydivide) {
           if(y % 2 == 0) {
             mesh.m_edges.push_back({index, index+(xdivide+1)});
             mesh.m_edges.push_back({index+(xdivide+1), index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index});
           } else {
             mesh.m_edges.push_back({index, index+(xdivide+1)+1});
             mesh.m_edges.push_back({index+(xdivide+1)+1, index+1});
             mesh.m_edges.push_back({index+1, index});
           }
           int e = mesh.m_edges.size()-1;
           mesh.m_triangles.push_back({e-2, e-1, e-0});
         }
#endif
         mesh.m_vertices.push_back(Vertex<VertexPayload> { location, payload } );

         index++;
      }
    }

#if 0
    int edges = mesh.m_edges.size();
    for(int i=0; i < edges; ++i) {
        mesh.m_edges.push_back({mesh.m_edges[i].toIndex,
                               mesh.m_edges[i].fromIndex});
    }
#endif

#if 0
    // Fill out neighbour information
    for(auto & e : mesh.m_edges) {
      mesh.m_vertices[e.fromIndex].neighbourIndices.push_back(e.toIndex);
      mesh.m_vertices[e.toIndex].neighbourIndices.push_back(e.fromIndex);
    }
#endif
    recreateEdgeNeighbourVertices(mesh);

    m_transformed = m_restshape = m_original = mesh;

#if 0
    // 4 handles just somewhere
    m_handles.clear();            
    m_handles.emplace_back(m_restshape.m_vertices.size()*0.2);
    m_handles.emplace_back(m_restshape.m_vertices.size()*0.4);
    m_handles.emplace_back(m_restshape.m_vertices.size()*0.6);
    m_handles.emplace_back(m_restshape.m_vertices.size()*0.8);
#endif

    //initialize();     
    return true;
  }

  Vertex<VertexPayload> createNewInterpolatedVertex(Triangulation<VertexPayload> & tri, int v1, int v2, float t)
  {
    Vertex<VertexPayload> & vx1 = tri.m_vertices[v1];
    Vertex<VertexPayload> & vx2 = tri.m_vertices[v2];


    return Vertex<VertexPayload> {
        (1-t)*vx1.location + t*vx1.location,
        (1-t)*vx1.payload + t*vx1.payload
      };
  }

  void recreateEdgeNeighbourVertices(Triangulation<VertexPayload> & tri)
  {
    std::map<int, int> hists;
#if 0
    // edge -> triangle index
    std::map< std::pair<int, int>, std::set<int> > toTriangle;
    for(int j=0; j < tri.m_triangles.size(); ++j) {
      auto & t = tri.m_triangles[j];
      for(int k=0; k < 3; ++k) {
        auto & e = tri.m_edges[t.edges[k]];
        toTriangle[std::make_pair(std::min(e.fromIndex, e.toIndex),
            std::max(e.fromIndex, e.toIndex))].insert(j);
      }
    }
    for(int i=0; i < tri.m_edges.size(); ++i) {
      auto & e = tri.m_edges[i];
      e.neighbourVertices.clear();

      auto & triangleIndices = toTriangle[std::make_pair(std::min(e.fromIndex, e.toIndex),
          std::max(e.fromIndex, e.toIndex))];

      std::set<int> vertices;
      for(auto & trIdx: triangleIndices) {
        auto & tr = tri.m_triangles.at(trIdx);
        for(int k=0; k < 3; ++k) {
          vertices.insert(tri.m_edges[tr.edges[k]].fromIndex);
          vertices.insert(tri.m_edges[tr.edges[k]].toIndex);
        }
        vertices.erase(e.fromIndex);
        vertices.erase(e.toIndex);

        hists[vertices.size()]++;
        for(auto & v: vertices)
          e.neighbourVertices.push_back(v);
      }
    }

#else
    for(int i=0; i < tri.m_edges.size(); ++i) {
      tri.m_edges[i].neighbourVertices.clear();

      std::vector<int> tris;
      for(int j=0; j < tri.m_triangles.size(); ++j) {
        if(tri.m_triangles[j].edges[0] == i ||
           tri.m_triangles[j].edges[1] == i ||
           tri.m_triangles[j].edges[2] == i) {
          tris.push_back(j);
        }
      }

      //hists[tris.size()]++;
      for(int j=0; j < tris.size(); ++j) {
        auto & t = tri.m_triangles[tris[j]];
        std::set<int> verts;
        for(int k=0; k < 3; ++k) {
          verts.insert(tri.m_edges[t.edges[k]].fromIndex);
          verts.insert(tri.m_edges[t.edges[k]].toIndex);
        }
        verts.erase(tri.m_edges[i].fromIndex);
        verts.erase(tri.m_edges[i].toIndex);
        assert(verts.size() == 1);

        tri.m_edges[i].neighbourVertices.push_back(*verts.begin());
      }
    }



    for(int i=0; i < tri.m_edges.size(); ++i) {
      auto & e1 = tri.m_edges[i];
      for(int j=i+1; j < tri.m_edges.size(); ++j) {
        auto & e2 = tri.m_edges[j];
        if(e1.fromIndex == e2.toIndex &&
           e1.toIndex == e2.fromIndex) {
          assert(e1.neighbourVertices.size() == 1);
          assert(e2.neighbourVertices.size() == 1);
          e1.neighbourVertices.push_back(e2.neighbourVertices.front());
          e2.neighbourVertices.push_back(e1.neighbourVertices.front());
        }
      }
    }

    for(int i=0; i < tri.m_edges.size(); ++i) {
      auto & e1 = tri.m_edges[i];
      hists[e1.neighbourVertices.size()]++;
    }
#endif
    Radiant::info("Edge neighbour counts: 0=%d, 1=%d, 2=%d, 3=%d",
                  hists[0], hists[1], hists[2], hists[3]);

  }


  void removeUnused()
  {
    removeUnusedEdges(m_original);
    removeUnusedVertices(m_original);

    removeUnusedEdges(m_restshape);
    removeUnusedVertices(m_restshape);

    removeUnusedEdges(m_transformed);
    removeUnusedVertices(m_transformed);

    recreateEdgeNeighbourVertices(m_original);
    recreateEdgeNeighbourVertices(m_restshape);
    recreateEdgeNeighbourVertices(m_transformed);
  }

  void removeUnusedVertices(Triangulation<VertexPayload> & tri)
  {
    std::set<int> usedVertices;
    for(auto & edge: tri.m_edges) {
      usedVertices.insert(edge.fromIndex);
      usedVertices.insert(edge.toIndex);
    }

    std::vector<int> offsets;
    offsets.reserve(tri.m_vertices.size());
    int offset = 0;
    for(int i=0; i < tri.m_vertices.size(); ) {
      if(usedVertices.count(i-offset) == 0) {
        tri.m_vertices.erase(tri.m_vertices.begin() + i);
        offsets.push_back(1);
        offset--;
      } else {
        offsets.push_back(offset);
        ++i;
      }
    }

    for(auto & edge : tri.m_edges) {
      assert(offsets.at(edge.fromIndex) <= 0);
      assert(offsets.at(edge.toIndex) <= 0);

      edge.fromIndex += offsets[edge.fromIndex];
      edge.toIndex += offsets[edge.toIndex];
    }

#if 0
    for(auto & v : tri.m_vertices) {
      for(int i=0; i < v.neighbourIndices.size(); ) {
        auto & idx = v.neighbourIndices[i];
        if(usedVertices.count(idx) == 0) {
          v.neighbourIndices.erase(v.neighbourIndices.begin() + i);
        } else {
          idx += offsets[idx];
          ++i;
        }
      }
    }

#endif
    for(int i=0; i < m_handles.size(); ++i) {
      if(usedVertices.count(m_handles[i]) == 0) {
        Radiant::error("erased handle %d", i);
        m_handles.erase(m_handles.begin() + i);
      } else {
        m_handles[i] += offsets[m_handles[i]];
        ++i;
      }
    }
  }   

  void removeUnusedEdges(Triangulation<VertexPayload> & tri)
  {
    std::set<int> usedEdges;
    for(auto & triangle: tri.m_triangles) {
      usedEdges.insert(triangle.edges[0]);
      usedEdges.insert(triangle.edges[1]);
      usedEdges.insert(triangle.edges[2]);
    }

    std::vector<int> offsets;
    offsets.reserve(tri.m_edges.size());
    int offset = 0;
    for(int i=0; i < tri.m_edges.size(); ) {
      if(usedEdges.count(i-offset) == 0) {
        tri.m_edges.erase(tri.m_edges.begin() + i);
        offsets.push_back(1);
        offset--;
      } else {
        offsets.push_back(offset);
        ++i;
      }
    }

    for(auto & triangle: tri.m_triangles) {
      assert(offsets[triangle.edges[0]] <= 0);
      assert(offsets[triangle.edges[1]] <= 0);
      assert(offsets[triangle.edges[2]] <= 0);

      triangle.edges[0] += offsets[triangle.edges[0]];
      triangle.edges[1] += offsets[triangle.edges[1]];
      triangle.edges[2] += offsets[triangle.edges[2]];
    }
  }

  void removeTriangle(int triIndex)
  {
    removeTriangle(m_original, triIndex);
    removeTriangle(m_restshape, triIndex);
    removeTriangle(m_transformed, triIndex);
  }

  void removeTriangle(Triangulation<VertexPayload> & tri, int triIndex)
  {
    assert(triIndex >= 0 && triIndex <= tri.m_triangles.size());
    tri.m_triangles.erase(tri.m_triangles.begin() + triIndex);
  }

  void initialize()
  {
    int vertices = m_transformed.m_vertices.size();
    int edges = m_transformed.m_edges.size();
    int constraints = m_handles.size();

    Radiant::info("Initialize # %d vertices, %d edges, %d triangles, %d constraints",
                  vertices, edges, int(m_transformed.m_triangles.size()), constraints);

    Radiant::TimeStamp ts = Radiant::TimeStamp::currentTime();

    m_a = Eigen::SparseMatrix<RealType>(2*edges + 2*constraints, 2*vertices);
    m_gks.clear();
    m_usedNeighbours.clear();

    /// @todo now this is hardcoded for max. 4 neighbour vertices (including 2 in the edge itself)
    Eigen::Matrix<RealType, 2, 8> t;
    t << -1,  0, 1, 0, 0, 0, 0, 0,
          0, -1, 0, 1, 0, 0, 0, 0;


    Eigen::Matrix<RealType, 8, 8> differ = Eigen::Matrix<RealType, 8, 8>::Zero();
    differ <<
         0,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,

        -1,  0,  1,  0,  0,  0,  0,  0,
         0, -1,  0,  1,  0,  0,  0,  0,

        -1,  0,  0,  0,  1,  0,  0,  0,
         0, -1,  0,  0,  0,  1,  0,  0,

        -1,  0,  0,  0,  0,  0,  1,  0,
         0, -1,  0,  0,  0,  0,  0,  1;


    std::set<int> handles;
    for(auto & h : m_handles)
      handles.insert(h);

    for(int k=0; k < edges; ++k) {
      Eigen::Matrix<RealType, 8, 2> g_k = Eigen::Matrix<RealType, 8, 2>::Zero();

      int fromIndex = m_restshape.m_edges[k].fromIndex;
      int toIndex = m_restshape.m_edges[k].toIndex;

#if 0
      std::set<int> neighbours1(m_restshape.m_vertices[fromIndex].neighbourIndices.begin(),
                                m_restshape.m_vertices[fromIndex].neighbourIndices.end());

      std::set<int> neighbours2(m_restshape.m_vertices[toIndex].neighbourIndices.begin(),
                                m_restshape.m_vertices[toIndex].neighbourIndices.end());

      neighbours1.erase(fromIndex);
      neighbours1.erase(toIndex);

      neighbours2.erase(fromIndex);
      neighbours2.erase(toIndex);


      std::vector<int> neighbourCandidates;
      std::set_union(neighbours1.begin(), neighbours1.end(),
                     neighbours2.begin(), neighbours2.end(),
                     std::back_inserter(neighbourCandidates));

      assert(neighbourCandidates.size() >= 2);

      const std::array<int,4> vertexIndices = {
        fromIndex, toIndex,
#if 1
        -1, -1,

#else
        *neighbourCandidates.begin(),
        *(++neighbourCandidates.begin())

#endif
      };
#else
      //Radiant::info("neighbours for edge %d = %d", k, int(m_restshape.m_edges[k].neighbourVertices.size()));
      const std::array<int,4> vertexIndices = {
        fromIndex, toIndex,
        m_restshape.m_edges[k].neighbourVertices.at(0),
        m_restshape.m_edges[k].neighbourVertices.size() > 1 ?
        m_restshape.m_edges[k].neighbourVertices.at(1) : -1
      };
#endif
      m_usedNeighbours.push_back(vertexIndices);

      int i=0;
      auto vOrigLocation = m_restshape.m_vertices[fromIndex].location;
      for(auto idx : vertexIndices) {
        if(idx == -1) break;
        assert(idx >= 0);
        assert(idx < m_restshape.m_vertices.size());

        auto diff = m_restshape.m_vertices[idx].location - vOrigLocation;
        g_k(i*2, 0) = diff[0];
        g_k(i*2, 1) = diff[1]; //m_restshape.m_vertices[idx].location[1];
        g_k(i*2+1, 0) = diff[1]; //m_restshape.m_vertices[idx].location[1];
        g_k(i*2+1, 1) = -diff[0]; //m_restshape.m_vertices[idx].location[0];

        i++;
      }

      auto e_k =
          m_restshape.m_vertices[toIndex].location -
          m_restshape.m_vertices[fromIndex].location;

      Eigen::Matrix<RealType, 2, 2> e_k_mat;
      e_k_mat << e_k[0],  e_k[1],
                 e_k[1], -e_k[0];

      m_gks.push_back((g_k.transpose()*g_k).inverse() * g_k.transpose() * differ);

      // Explicit inverse should be okay for small matrix.
      // Use SVD or something if there's problems.
      Eigen::Matrix<RealType, 2, 8> h_mat =
          //-e_k_mat * m_gks.back();
          t - (e_k_mat * m_gks.back());

      i = 0;

      for(auto idx : vertexIndices) {
        if(idx == -1)
          break;

#if 0
        m_a.insert(k*2, 2*idx) = h_mat(0, 2*i);
        m_a.insert(k*2, 2*idx+1) = h_mat(0, 2*i+1);
        m_a.insert(k*2+1, 2*idx) = h_mat(1, 2*i);
        m_a.insert(k*2+1, 2*idx+1) = h_mat(1, 2*i+1);
#else
        if(h_mat(0, 2*i) != 0)
          m_a.coeffRef(k*2, 2*idx) = h_mat(0, 2*i);
        if(h_mat(0, 2*i+1) != 0)
          m_a.coeffRef(k*2, 2*idx+1) = h_mat(0, 2*i+1);
        if(h_mat(1, 2*i) != 0)
          m_a.coeffRef(k*2+1, 2*idx) = h_mat(1, 2*i);
        if(h_mat(1, 2*i+1) != 0)
          m_a.coeffRef(k*2+1, 2*idx+1) = h_mat(1, 2*i+1);
#endif
        i++;
      }
      //std::cout << h_mat << std::endl;
    }


    for(int i=0; i < constraints; ++i) {
      m_a.insert(2*edges+2*i, 2*m_handles[i]) = m_constraintWeight;
      m_a.insert(2*edges+2*i+1, 2*m_handles[i]+1) = m_constraintWeight;
    }

    m_a.makeCompressed();
    int nonZeros = m_a.nonZeros();

    m_a_t = m_a.transpose();
    m_a_t_a = m_a_t * m_a;

    int nonZerosAaT = m_a_t_a.nonZeros();

    std::cout << "nnz A=" << nonZeros << ", A^T*A=" << nonZerosAaT << ", total " << m_a.size() << std::endl;

    //std::cout << m_a << std::endl;

    //std::cout << (a.transpose() * a) << std::endl;

    //m_svd.compute(m_a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //m_qr.compute(m_a_t_a);
    m_ldlt.compute(m_a_t_a);

    //m_lscg.compute(m_a);
    m_lscg.compute(m_a_t_a);
    m_lscg.setMaxIterations(5000);
    m_lscg.setTolerance(1e-10f);

    if(m_v_1.rows() != 2*vertices || m_v_1.cols() != 1) {
      m_v_1.resize(2*vertices, 1);

      for(int i=0; i < vertices; ++i) {
        m_v_1.coeffRef(2*i, 0) = m_restshape.m_vertices[i].location[0];
        m_v_1.coeffRef(2*i+1, 0) = m_restshape.m_vertices[i].location[1];
      }
    }
#if 0
    m_b = Eigen::SparseMatrix<RealType>(2*edges + 2*constraints, 1);
#else
    m_b.resize(2*edges + 2*constraints, 1);
    for(int i=0; i < edges; ++i) {
       auto diff =
           m_restshape.m_vertices[m_restshape.m_edges[i].toIndex].location -
           m_restshape.m_vertices[m_restshape.m_edges[i].fromIndex].location;
       m_b(2*i, 0) = 0*diff[0];
       m_b(2*i+1, 0) = 0*diff[1];
    }     
#endif
    m_a_2 = Eigen::SparseMatrix<RealType>(edges + constraints, vertices);
    for(int i=0; i < edges; ++i) {
      m_a_2.insert(i, m_restshape.m_edges[i].fromIndex) = -1;
      m_a_2.insert(i, m_restshape.m_edges[i].toIndex) = 1;
    }

    for(int i=0; i < constraints; ++i) {
      m_a_2.insert(edges+i, m_handles[i]) = m_constraintWeight;
    }

    m_a_2.makeCompressed();

    m_a_t_2 = m_a_2.transpose();
    m_a_t_a_2 = m_a_t_2 * m_a_2;

    m_ldlt_2.compute(m_a_t_a_2);

    m_lscg_2.compute(m_a_t_a_2);
    m_lscg_2.setMaxIterations(50);
    m_lscg_2.setTolerance(1e-10f);

    if(m_v_2.cols() != 2 || m_v_2.rows() != vertices) {
      m_v_2.resize(vertices, 2);

      for(int i=0; i < vertices; ++i) {
        m_v_2(i, 0) = m_transformed.m_vertices[i].location[0];
        m_v_2(i, 1) = m_transformed.m_vertices[i].location[1];
      }
    }      

    m_b_2.resize(edges+constraints, 2);

    Radiant::info("Initialize took %lf ms", 1000.0*(Radiant::TimeStamp::currentTime()-ts).secondsD());
  }
#if 0
  void initializeTrivial()
  {
    assert(false);

    int vertices = m_transformed.m_vertices.size();
    int edges = m_transformed.m_edges.size();
    int constraints = m_handles.size();

    m_a = Eigen::SparseMatrix<RealType>(edges + constraints, vertices);
    //= Eigen::MatrixXf::Zero(edges + constraints, vertices);
    for(int i=0; i < edges; ++i) {
      m_a.insert(i, m_restshape.m_edges[i].fromIndex) = -1;
      m_a.insert(i, m_restshape.m_edges[i].toIndex) = 1;
    }

    for(int i=0; i < constraints; ++i) {
      m_a.insert(edges+i, m_handles[i]) = m_constraintWeight;
    }

    //std::cout << (a.transpose() * a) << std::endl;

    //m_svd.compute(m_a, Eigen::ComputeThinU | Eigen::ComputeThinV);

    m_lscg.compute(m_a);

    m_b.resize(edges + constraints, 2);
    for(int i=0; i < edges; ++i) {
       auto diff =
           m_restshape.m_vertices[m_restshape.m_edges[i].toIndex].location -
           m_restshape.m_vertices[m_restshape.m_edges[i].fromIndex].location;
       m_b(i, 0) = diff[0];
       m_b(i, 1) = diff[1];
    }

  }
#endif

  std::vector<int> components(const Triangulation<VertexPayload> & tri, int & componentCount)
  {
    std::vector<int> colors(tri.m_vertices.size(), -1);
    int currentColor = 0;

    std::stack<int> vertices;

    // adjacency list
    std::vector<std::vector<int>> neighbours(tri.m_vertices.size());
    for(auto & e : tri.m_edges) {
      neighbours[e.fromIndex].push_back(e.toIndex);
    }

    for(int i=0; i < tri.m_vertices.size(); ++i) {
      if(colors[i] >= 0)
        continue;

      vertices.push(i);
      while(!vertices.empty()) {
        int v = vertices.top();
        vertices.pop();

        colors[v] = currentColor;

        for(int idx: neighbours[v]) {
          if(colors[idx] < 0)
           vertices.push(idx);
        }
      }
      currentColor++;
    }

    componentCount = currentColor;
    return colors;
  }


  int nearestHandleIndex(Nimble::Vector2 location, float threshold)
  {
    int index = -1;
    float mindist = std::numeric_limits<float>::max();
    for(auto & h : m_handleIdentifiers) {
      float d = (m_transformed.m_vertices[m_handles[h.second]].location - location).length();
      if(d < mindist) {
        index = h.first;
        mindist = d;
      }
    }
    return mindist < threshold ? index : -1;
  }

  int nearestTransformedVertexIndex(Nimble::Vector2 location)
  {
    int index = -1;
    float mindist = std::numeric_limits<float>::max();
    for(int i=0; i < m_transformed.m_vertices.size(); ++i) {
      float d = (m_transformed.m_vertices[i].location - location).length();
      if(d < mindist) {
        index = i;
        mindist = d;
      }
    }
    return index;
  }

  void moveHandle(int handleIndex, Nimble::Vector2 to)
  {
    if(m_handleIdentifiers.count(handleIndex) == 0) {
      Radiant::error("Handle %d does not exist", handleIndex);
      return;
    }

    int vIdx = m_handles.at(m_handleIdentifiers[handleIndex]);
    if(m_transformed.m_vertices[vIdx].location == to)
      return;

    m_transformed.m_vertices[vIdx].location = to;

    m_dirty = true;
  }

  int addHandle(Nimble::Vector2 at)
  {
    int index = nearestTransformedVertexIndex(at);
    if(index == -1)
      return -1;

    m_handles.push_back(index);
    m_handleIdentifiers[m_handleIndex] = m_handles.size()-1;
    m_handleIndex++;

    /// @todo just make dirty?
    initialize();
    m_dirty = true;
    return m_handleIndex-1;
  }

  void removeHandle(int handleId)
  {
    auto it = m_handleIdentifiers.find(handleId);
    if(it == m_handleIdentifiers.end())
      return;

    int handleIndex = it->second;

    m_handles.erase(m_handles.begin() + handleIndex);
    m_handleIdentifiers.erase(handleId);
    for(auto & h : m_handleIdentifiers) {
      if(h.second > handleIndex)
        h.second--;
    }
    initialize();
    m_dirty = true;
  }


  // returns true if something was actually updated
  bool update()
  {
    if(!m_dirty)
      return false;

    if(m_handles.size() < 1) {
      m_transformed = m_restshape;
      return true;
    }

    Radiant::TimeStamp ts = Radiant::TimeStamp::currentTime();
    int vertices = m_transformed.m_vertices.size();
    int edges = m_transformed.m_edges.size();
    int constraints = m_handles.size();

    Eigen::Matrix<RealType, Eigen::Dynamic, 1> v;
    if(m_handles.size() <= 1) {
      v.resize(2*vertices, 1);
      for(int i=0; i < vertices; ++i) {
        v.coeffRef(2*i, 0) = m_restshape.m_vertices[i].location[0];
        v.coeffRef(2*i+1, 0) = m_restshape.m_vertices[i].location[1];
      }
      m_v_1 = v;
    } else {
#if 1
      //Eigen::SparseMatrix<RealType> vGuess(2*vertices, 1);
#else
      Eigen::Matrix<RealType, Eigen::Dynamic, 1> vGuess;
      vGuess.resize(2*vertices, 1);
      for(int i=0; i < vertices; ++i) {
        vGuess.coeffRef(2*i, 0) = m_transformed.m_vertices[i].location[0];
        vGuess.coeffRef(2*i+1, 0) = m_transformed.m_vertices[i].location[1];
      }
#endif

      for(int i=0; i < constraints; ++i) {
        auto vec = m_transformed.m_vertices[m_handles[i]].location;
        m_b.coeffRef(2*edges+2*i, 0) = m_constraintWeight*vec[0];
        m_b.coeffRef(2*edges+2*i+1, 0) = m_constraintWeight*vec[1];
      }

      //Eigen::Matrix<RealType, Eigen::Dynamic, 1> v = m_lscg.solveWithGuess(m_b, m_v_1); //solveWithGuess(m_b, vGuess);
      //Eigen::Matrix<RealType, Eigen::Dynamic, 1> v = m_lscg.solveWithGuess(m_a_t*m_b, m_v_1); //solveWithGuess(m_b, vGuess);
      //Eigen::Matrix<RealType, Eigen::Dynamic, 1> v = m_qr.solve(m_a_t*m_b);
#ifdef ITERATIVE_SOLVE
      v = m_lscg.solveWithGuess(m_a_t*m_b, m_v_1);
#else
      v = m_ldlt.solve(m_a_t*m_b);
#endif

      m_v_1 = v;
    }

    updateSecondStep(v);
    //std::cout << "Took " << (1000.0*(Radiant::TimeStamp::currentTime()-ts).secondsD()) << "ms" << std::endl;

    m_dirty = false;
    //Eigen::Matrix<RealType, Eigen::Dynamic, 1> v = m_qr.solve(m_b);
#if 0
    //Eigen::Matrix<RealType, Eigen::Dynamic, Eigen::Dynamic> v1 = m_svd.solve(m_b);
    std::cout << "iters: " << m_lscg.iterations() << "/" << m_lscg.maxIterations() << " ";
    std::cout << "tolerance: " << m_lscg.tolerance() << " ";
    std::cout << "errors: " << m_lscg.error() << ", ";
    std::cout << (m_a*v - m_b).norm() << std::endl;
    //std::cout << (m_a*v1 - m_b).norm() << std::endl;
    //v = v1;
#endif
    return true;
  }

  void updateSecondStep(const Eigen::Matrix<RealType, Eigen::Dynamic, 1> & v)
  {
    int vertices = m_transformed.m_vertices.size();
    int edges = m_transformed.m_edges.size();
    int constraints = m_handles.size();

    /*
    Eigen::Matrix<RealType, Eigen::Dynamic, 2> b;
    vGuess.resize(vertices, 2);
    for(int i=0; i < vertices; ++i) {
      vGuess(i, 0) = m_transformed.m_vertices[i].location[0];
      vGuess(i, 1) = m_transformed.m_vertices[i].location[1];
    }
    b.resize(edges + constraints, 2);
*/

    for(int i=0; i < edges; ++i) {
      Eigen::Matrix<RealType, 8, 1> vs;

      for(int j=0; j < 4; ++j) {
        int nIndex = m_usedNeighbours[i][j];
        if(nIndex == -1) {
          vs(2*j, 0) = 0;
          vs(2*j+1, 0)= 0;
        } else {
          vs(2*j, 0) = v(2*nIndex);
          vs(2*j+1, 0)= v(2*nIndex+1);
        }
      }
      Eigen::Matrix<RealType, 2, 1> c = m_gks.at(i) * vs;

      c.normalize();
      //c = c / c.norm();

      Eigen::Matrix<RealType, 2, 2> t;
      t << c(0), c(1),
           -c(1), c(0);
/*
      std::cout << "c_" << i << " = " << c.transpose() << " @ " <<
                   m_restshape.m_vertices[m_restshape.m_edges[i].fromIndex].location[0]
          << ", " << m_restshape.m_vertices[m_restshape.m_edges[i].fromIndex].location[1]
                << std::endl;
*/
      auto diff = m_restshape.m_vertices[m_restshape.m_edges[i].toIndex].location -
          m_restshape.m_vertices[m_restshape.m_edges[i].fromIndex].location;

      Eigen::Matrix<RealType, 2,1> loc;
      loc << diff[0], diff[1];
      loc = t*loc;

      m_b_2(i, 0) = loc(0);
      m_b_2(i, 1) = loc(1);
    }

    for(int i=0; i < constraints; ++i) {
      auto vec = m_transformed.m_vertices[m_handles[i]].location;
      m_b_2(edges+i, 0) = m_constraintWeight*vec[0];
      m_b_2(edges+i, 1) = m_constraintWeight*vec[1];
    }


#ifdef ITERATIVE_SOLVE
    Eigen::Matrix<RealType, Eigen::Dynamic, 2> v_ = m_lscg_2.solveWithGuess(m_a_t_2*m_b_2, m_v_2);
#else
    Eigen::Matrix<RealType, Eigen::Dynamic, 2> v_ = m_ldlt_2.solve(m_a_t_2*m_b_2);
#endif

    m_v_2 = v_;

    for(int i=0; i < vertices; ++i) {
      m_transformed.m_vertices[i].location[0] = v_(i, 0);
      m_transformed.m_vertices[i].location[1] = v_(i, 1);
    }

#if 0
    std::cout << "iters: " << m_lscg_2.iterations() << "/" << m_lscg_2.maxIterations() << " ";
    std::cout << "tolerance: " << m_lscg_2.tolerance() << " ";
    std::cout << "errors: " << m_lscg_2.error() << ", ";
    std::cout << (m_a_2*v_ - m_b_2).norm() << std::endl;
    //std::cout << (m_a*v1 - m_b).norm() << std::endl;
    //v = v1;
#endif

    Eigen::Matrix<RealType, Eigen::Dynamic, 2> errors = m_a_2*v_ - m_b_2;
    m_errors = errors.rowwise().norm();
    Radiant::info("Error=%f, rows=%d, vertices=%d, edges=%d", errors.norm(), int(errors.rows()), vertices, edges);
  }
#if 0
  bool updateTrivial()
  {
    if(!m_dirty)
      return true;

    Radiant::TimeStamp ts = Radiant::TimeStamp::currentTime();
    int vertices = m_transformed.m_vertices.size();
    int edges = m_transformed.m_edges.size();
    int constraints = m_handles.size();

    Eigen::Matrix<RealType, Eigen::Dynamic, 2> vGuess;
    vGuess.resize(vertices, 2);
    for(int i=0; i < vertices; ++i) {
      vGuess(i, 0) = m_transformed.m_vertices[i].location[0];
      vGuess(i, 1) = m_transformed.m_vertices[i].location[1];
    }

    for(int i=0; i < constraints; ++i) {
      auto vec = m_transformed.m_vertices[m_handles[i]].location;
      m_b(edges+i, 0) = m_constraintWeight*vec[0];
      m_b(edges+i, 1) = m_constraintWeight*vec[1];
    }

    m_lscg.setTolerance(1e-12f);
    Eigen::Matrix<RealType, Eigen::Dynamic, Eigen::Dynamic> v = m_lscg.solveWithGuess(m_b, vGuess);
#if 0
    Eigen::MatrixXf v1 = m_svd.solve(m_b);
    std::cout << "iters: " << m_lscg.iterations() << "/" << m_lscg.maxIterations() << " ";
    std::cout << "tolerance: " << m_lscg.tolerance() << " ";
    std::cout << "errors: " << m_lscg.error() << ", ";
    std::cout << (m_a*v - m_b).norm() << std::endl;
    std::cout << (m_a*v1 - m_b).norm() << std::endl;
#endif
    for(int i=0; i < vertices; ++i) {
      m_transformed.m_vertices[i].location[0] = v(i, 0);
      m_transformed.m_vertices[i].location[1] = v(i, 1);
    }
    m_dirty = false;

    Radiant::info("Took %lf ms", 1000.0*(Radiant::TimeStamp::currentTime()-ts).secondsD());

    return true;
  }
#endif

  const float m_constraintWeight = 100.0f;
//  private:
  Triangulation<VertexPayload> m_original;
  Triangulation<VertexPayload> m_restshape;
  Triangulation<VertexPayload> m_transformed;

  /// @todo free-form handles, not tied to vertices
  std::vector<size_t> m_handles;

  // handle identifier -> handle index
  std::map<size_t, size_t> m_handleIdentifiers;
  int m_handleIndex = 0;

  Eigen::SparseMatrix<RealType> m_a;
  // Transpose of m_a. Might not be necessary to store it at all.
  // Could use m_a.transpose() in products and expression templates
  // in eigen will take care of not actually creating extra temporary copies
  Eigen::SparseMatrix<RealType> m_a_t;
  /// m_a.transpose() * m_a
  Eigen::SparseMatrix<RealType> m_a_t_a;

  Eigen::SparseMatrix<RealType> m_a_2;
  Eigen::SparseMatrix<RealType> m_a_t_2;
  Eigen::SparseMatrix<RealType> m_a_t_a_2;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<RealType>> m_ldlt_2;

  //Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<RealType>> m_lscg_2;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<RealType>> m_lscg_2;
  Eigen::Matrix<RealType, Eigen::Dynamic, 2> m_b_2;

  Eigen::SparseQR<Eigen::SparseMatrix<RealType>, Eigen::COLAMDOrdering<int>> m_qr;

  //Eigen::LeastSquaresConjugateGradient<Eigen::SparseMatrix<RealType>> m_lscg;
  //Eigen::ConjugateGradient<Eigen::SparseMatrix<RealType>> m_lscg;
  Eigen::ConjugateGradient<Eigen::SparseMatrix<RealType>> m_lscg;

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<RealType>> m_ldlt;

  Eigen::Matrix<RealType, Eigen::Dynamic, 1> m_b;

  //Eigen::SparseMatrix<RealType> m_b;

  //Eigen::SparseMatrix<RealType> m_v_1;
  Eigen::Matrix<RealType, Eigen::Dynamic, 1> m_v_1;
  Eigen::Matrix<RealType, Eigen::Dynamic, 2> m_v_2;

  std::vector<Eigen::Matrix<RealType, 2, 8>,
    Eigen::aligned_allocator<Eigen::Matrix<RealType, 2, 8>>> m_gks;

  std::vector<std::array<int,4>> m_usedNeighbours;

  Eigen::Matrix<RealType, Eigen::Dynamic, 1> m_errors;
private:
  bool m_dirty = true;
};

class ShapeManipulationWidget::D
{
public:
  D(MultiWidgets::Widget & host)
    : m_persistTransformedShape(&host, "persist-transformed-shape", true)
    , m_renderDetails(&host, "render-details", true)
    , m_cutThreshold(&host, "cut-threshold", 2.0f)
  {
  }

  ArapShape<TextureCoordinate> m_shape;

  std::map<MultiTouch::TrackedObject::Id, size_t> m_grabbedHandles;

  Luminous::Image m_image;
  Valuable::AttributeBool m_persistTransformedShape;
  Valuable::AttributeBool m_renderDetails;
  Valuable::AttributeFloat m_cutThreshold;
};

ShapeManipulationWidget::ShapeManipulationWidget()
  : MultiWidgets::Widget()
  , m_d(new D(*this))
{
  setCSSId("shape");

  /// FIXME: don't hardcode filename/construction parameters
  bool ok = m_d->m_image.read("pizza.png", true);
  if(!ok)
    Radiant::error("Failed to load image");

  m_d->m_shape.constructFromUnitRect(35, 35);

  {
    Nimble::Matrix3 toImageNormalized = fromTriangulation;

    for(int i=0; i < m_d->m_shape.m_restshape.m_triangles.size();) {
      auto t = m_d->m_shape.m_restshape.m_triangles[i];

      Nimble::Vector2 locs[] = {
        m_d->m_shape.m_restshape.m_vertices[m_d->m_shape.m_restshape.m_edges[t.edges[0]].fromIndex].location,
        m_d->m_shape.m_restshape.m_vertices[m_d->m_shape.m_restshape.m_edges[t.edges[1]].fromIndex].location,
        m_d->m_shape.m_restshape.m_vertices[m_d->m_shape.m_restshape.m_edges[t.edges[2]].fromIndex].location
      };

      bool keep = false;
      for(auto & loc: locs) {
        auto loc_ = toImageNormalized.project(loc);
        /// pixelAlpha() actually want coordinates in [0, 1[, not [0,1]
        loc_.clamp(0, 0.999999);
        keep = m_d->m_image.pixelAlpha(loc_) > 0;
        if(keep) break;
      }

      if(!keep) {
        m_d->m_shape.removeTriangle(i);
      } else {
        ++i;
      }
    }
  }

  m_d->m_shape.removeUnused();

  m_d->m_shape.initialize();

  /*
  eventAddListenerBd("single-tap", [this](Radiant::BinaryData & bd){
    bool ok = false;
    Nimble::Vector2 loc = bd.readVector2Float32(&ok);
    loc.descale(size().toVector());
    loc = toTriangulation.project(loc);
    int existingIndex = m_d->m_shape.nearestHandleIndex(loc, 0.05f*s_scale.length());
    if(existingIndex >= 0) {
      if(m_d->m_shape.m_handles.size() >= 1) {
        m_d->m_shape.removeHandle(existingIndex);
      }
    } else {
      m_d->m_shape.addHandle(loc);
    }
  });
  */
}

void ShapeManipulationWidget::renderContent(Luminous::RenderContext & r) const
{
  //MultiWidgets::Widget::renderContent(r);

  r.pushTransformRightMul(Nimble::Matrix3::makeScale(width(), height()));
  const auto & triangulation = m_d->m_shape.m_transformed;

  Luminous::Style myStyle;
  // What does the style affect? color and shader is given separately to drawPrimitiveT?
  Radiant::Color color(1, 1, 1, 1);
  myStyle.setFillColor(color);
  myStyle.setFillProgram(r.texShader());
  myStyle.setTexture(m_d->m_image.texture());

  r.pushTransformRightMul(fromTriangulation);
  float invScale = 1.0f/r.approximateScaling();

  const int triangles = triangulation.m_triangles.size();
  auto b = r.drawPrimitiveT<Luminous::BasicVertexUV, Luminous::BasicUniformBlock>(
        Luminous::PRIMITIVE_TRIANGLE, 0, 3*triangles,
        r.texShader(), color, 1.f, myStyle);


  for(int i=0; i < triangles; ++i) {
    auto & t = triangulation.m_triangles[i];
    auto & v1 = triangulation.m_vertices[triangulation.m_edges[t.edges[0]].fromIndex];
    auto & v2 = triangulation.m_vertices[triangulation.m_edges[t.edges[1]].fromIndex];
    auto & v3 = triangulation.m_vertices[triangulation.m_edges[t.edges[2]].fromIndex];

    b.vertex[i*3].location = v1.location;
    b.vertex[i*3].texCoord = v1.payload;

    b.vertex[i*3+1].location = v2.location;
    b.vertex[i*3+1].texCoord = v2.payload;

    b.vertex[i*3+2].location = v3.location;
    b.vertex[i*3+2].texCoord = v3.payload;
  }


  Luminous::Style vertexStyle;

  vertexStyle.setFillColor(1, 1, 0, 0.8f);
  Luminous::Style handleStyle;
  handleStyle.setFillColor(1, 0, 1, 0.8f);

  Luminous::Style edgeStyle;
  edgeStyle.setStrokeColor(1, 0, 0, 0.1f);
  /// this seems to be defined in screen space
  edgeStyle.setStrokeWidth(2);


#if 0
  auto & errors = m_d->m_shape.m_errors;
  if(m_d->m_renderDetails) {      
    Luminous::Style neighbourStyle;
    neighbourStyle.setStrokeColor(1, 1, 1, 0.4f);
    neighbourStyle.setStrokeWidth(2);
    for(int i=0; i < errors.rows() - m_d->m_shape.m_handles.size(); ++i) {
      float size = errors(i); //.row(i).norm();
      size += 2;
      if(size >= 1) {
        edgeStyle.setStrokeWidth(size);
        auto & e = triangulation.m_edges.at(i);
        bool onBorder = e.neighbourVertices.size() == 1;
        edgeStyle.setStrokeColor(onBorder ? 1 : 0, onBorder ? 0 : 1, 0, size/m_d->m_cutThreshold);
        r.drawLine(triangulation.m_vertices[e.fromIndex].location,
            triangulation.m_vertices[e.toIndex].location, edgeStyle);

        Nimble::Vector2 middle = 0.3f*triangulation.m_vertices[e.fromIndex].location+
          0.7f*triangulation.m_vertices[e.toIndex].location;


        for(auto & vIndex : e.neighbourVertices) {
          Nimble::Vector2 to = triangulation.m_vertices[vIndex].location;
          to = middle + 0.4f * (to - middle);
          r.drawLine(middle, to,
              neighbourStyle);
        }
      }
    }
  }
#endif
  for(auto & h : m_d->m_shape.m_handles) {
    r.drawCircle(triangulation.m_vertices[h].location, invScale*15, handleStyle);
  }

  if(m_d->m_renderDetails) {
    for(auto & v : triangulation.m_vertices) {
      r.drawCircle(v.location, invScale*5, vertexStyle);
    }

    for(auto & e : triangulation.m_edges) {
      r.drawLine(triangulation.m_vertices[e.fromIndex].location,
          triangulation.m_vertices[e.toIndex].location, edgeStyle);
    }
  }

  r.popTransform();

}

/// @todo implement
bool ShapeManipulationWidget::isInside(Nimble::Vector2 v) const
{
  return MultiWidgets::Widget::isInside(v);
}

/// @todo implement
Nimble::Rect ShapeManipulationWidget::boundingRect() const
{
  return MultiWidgets::Widget::boundingRect();
}

void ShapeManipulationWidget::update(const MultiWidgets::FrameInfo &frameInfo)
{
  MultiWidgets::Widget::update(frameInfo);

  if(frameInfo.isLastUpdate()) {
    m_d->m_shape.update();

    // Check if there's a boundary edge that has too much stress
    Radiant::Timer timer;
    std::set<std::pair<float, int>> edgesToRemove;
    // We only care about topology here so just use the original mesh
    auto & tri = m_d->m_shape.m_original;
    for(int i=0; i < m_d->m_shape.m_errors.rows(); ++i) {
      if(m_d->m_shape.m_errors(i) > m_d->m_cutThreshold) {
        auto & edge = tri.m_edges[i];
        if(edge.neighbourVertices.size() != 1)
          continue; // Not an boundary edge

        edgesToRemove.emplace(-m_d->m_shape.m_errors(i), i);
      }
    }

    bool removed = false;
    for(auto & toRemove : edgesToRemove) {

      int edgeIndex = toRemove.second;
      Radiant::info("remove edge %d with stress %f",
                    toRemove.second,
                    -toRemove.first);

      if(!std::isfinite(toRemove.first))
        return;

      int triangleIndex = -1;
      int edgeIndexInTriangle = -1;


      // There should be only one triangle that this edge belongs to, find it
      for(int i=0; i < tri.m_triangles.size(); ++i) {
        edgeIndexInTriangle = tri.m_triangles[i].indexForEdgeIndex(edgeIndex);
        if(edgeIndexInTriangle != -1) {
          triangleIndex = i;
          break;
        }
      }

      assert(triangleIndex != -1);

      // Duplicate vertex
      int newIndex = tri.m_vertices.size();
      int oldIndex = tri.m_edges[edgeIndex].fromIndex;

      // If the vertex we are about to duplicate doesn't belong to any other triangle,
      // don't do it.
      // Should have at least two edges that start from it
      int fromDegree = 0;
      for(auto & e : tri.m_edges) {
        if(e.fromIndex == oldIndex)
          fromDegree++;
      }

      if(fromDegree < 2)
        continue;

      // If the duplicated vertex is constrained, move the constraint
      // to the new one
      auto it = std::find(m_d->m_shape.m_handles.begin(),
                   m_d->m_shape.m_handles.end(), oldIndex);
      if(it != m_d->m_shape.m_handles.end()) {
        //*it = newIndex;
        continue;
      }

#if 1
      auto newVertex = tri.m_vertices[tri.m_edges[edgeIndex].fromIndex];
      tri.m_vertices.push_back(newVertex);

      int edgeIndexToFix = tri.m_triangles[triangleIndex].edges[(edgeIndexInTriangle+2)%3];

      auto & boundaryEdge = tri.m_edges[edgeIndexToFix];
      assert(boundaryEdge.toIndex == oldIndex);

      boundaryEdge.neighbourVertices.clear();
      boundaryEdge.neighbourVertices.push_back(tri.m_edges[edgeIndex].toIndex);

      // Find a twin edge for the edge that we make boundary (if it exists)
      // No need to do this, we'll just recreate neighbour indices completely instead
#if 0
      for(int i=0; i < tri.m_edges.size(); ++i) {
        auto & edge = tri.m_edges[i];
        if(edge.fromIndex == boundaryEdge.toIndex &&
           edge.toIndex == boundaryEdge.fromIndex) {

          int neighbourIndexToRemove = tri.m_edges[edgeIndex].toIndex;

          auto it = std::find(edge.neighbourVertices.begin(), edge.neighbourVertices.end(),
                    neighbourIndexToRemove);

          if(it != edge.neighbourVertices.end())
            edge.neighbourVertices.erase(it);

          break;
        }
      }
#endif
      boundaryEdge.toIndex = newIndex;
      tri.m_edges[edgeIndex].fromIndex = newIndex;
#endif
      m_d->m_shape.recreateEdgeNeighbourVertices(m_d->m_shape.m_original);

      // create the new vertex in other meshes (cannot copy since location can be different)
      m_d->m_shape.m_restshape.m_vertices.push_back(m_d->m_shape.m_restshape.m_vertices[oldIndex]);
      m_d->m_shape.m_transformed.m_vertices.push_back(m_d->m_shape.m_transformed.m_vertices[oldIndex]);

      // topology can be just copied
      m_d->m_shape.m_restshape.m_triangles = tri.m_triangles;
      m_d->m_shape.m_restshape.m_edges = tri.m_edges;

      m_d->m_shape.m_transformed.m_triangles = tri.m_triangles;
      m_d->m_shape.m_transformed.m_edges = tri.m_edges;

      //m_d->m_shape.recreateEdgeNeighbourVertices(m_d->m_shape.m_restshape);
      //m_d->m_shape.recreateEdgeNeighbourVertices(m_d->m_shape.m_transformed);

      m_d->m_shape.initialize();
      m_d->m_shape.update();
      // Only remove one edge per update
      break;
    }

    // Now check if the graph became disconnected
    // If so, make a separate shape out of all connected components
#if 0
    static int lastCount = 0;
    int count = 0;
    std::vector<int> colors = m_d->m_shape.components(m_d->m_shape.m_original, count);
    if(count > 1) {
      if(count != lastCount) {
        Radiant::error("Disconnected graph, components %d", count);
        //m_d->m_shape.m_restshape = m_d->m_shape.m_transformed;
        lastCount = count;
      }
    }
#endif
//      std::cout << "Widget update took " << (1000.0*timer.time()) << "ms" << std::endl;
  }
}

void ShapeManipulationWidget::processFingers(MultiWidgets::GrabManager &gm, const MultiWidgets::FingerArray & fingers, float dt)
{
  gm.pushTransformLeftMul(toTriangulation *
                          Nimble::Matrix3::makeScale(1.0f/width(), 1.0f/height()));


  for(MultiTouch::Finger finger : fingers) {
    if(m_d->m_grabbedHandles.count(finger.id()) == 0) {
      auto loc = gm.project(finger.tipLocation());
      int index = m_d->m_shape.nearestHandleIndex(loc, 0.05f*s_scale.length());
      if(index != -1)
        m_d->m_grabbedHandles[finger.id()] = index;
      else {
        index = m_d->m_shape.addHandle(loc);
        if(index != -1)
          m_d->m_grabbedHandles[finger.id()] = index;
      }
    }
  }

  for(auto it = m_d->m_grabbedHandles.begin(); it != m_d->m_grabbedHandles.end(); ) {
    MultiTouch::Finger finger = gm.findFinger(it->first);
    if(finger.isNull()) {
      m_d->m_shape.removeHandle(it->second);
      it = m_d->m_grabbedHandles.erase(it);
    } else {
      auto loc = gm.project(finger.tipLocation());
      m_d->m_shape.moveHandle(it->second, loc);
      ++it;
    }
  }

  gm.popTransform();
}

void ShapeManipulationWidget::interactionEnd(MultiWidgets::GrabManager &gm)
{
  for(auto it = m_d->m_grabbedHandles.begin(); it != m_d->m_grabbedHandles.end(); ++it) {
    m_d->m_shape.removeHandle(it->second);
  }
  m_d->m_grabbedHandles.clear();

  assert(m_d->m_shape.m_handles.size() == 0);
  if(m_d->m_persistTransformedShape) {
    m_d->m_shape.m_restshape = m_d->m_shape.m_transformed;
  } else {
    m_d->m_shape.m_transformed = m_d->m_shape.m_restshape;
  }
}


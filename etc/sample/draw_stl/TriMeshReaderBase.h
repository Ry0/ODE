#ifndef __ODE_UTILS_TRIMESHREADER_BASE_H
#define __ODE_UTILS_TRIMESHREADER_BASE_H

namespace ode_utils {

template <typename V_TYPE, typename IDX_TYPE>
class TriMeshReaderBase
{
public:
    typedef V_TYPE   vertex_type;
    typedef IDX_TYPE index_type;

protected:
    int m_vertexCount;
    int m_indexCount;
    vertex_type  *m_vertices;
    index_type   *m_indices;

    bool m_complete;

public:
    TriMeshReaderBase( const char* file_name )
        : m_vertexCount(-1)
        , m_indexCount(-1)
        , m_vertices(NULL)
        , m_indices(NULL)
        , m_complete(false)
    {
    }

    virtual ~TriMeshReaderBase()
    {
        if (m_vertices) delete [] m_vertices;
        m_vertices = NULL;
        if (m_indices) delete [] m_indices;
        m_indices = NULL;
    }

protected:
    virtual bool createMeshData( const char* file_name ) = 0;

public:

    virtual bool isCompleted() {
        if (!m_complete) return false;
        if (m_vertexCount < 1)  return false;
        if (m_indexCount < 1)  return false;
        if (!m_vertices) return false;
        if (!m_indices)  return false;
        return true;
    }

    virtual const char* message() = 0;

    const vertex_type* getVertices() {
        return (const vertex_type*)m_vertices;
    }

    const index_type* getIndices() {
        return (const index_type*)m_indices;
    }

    int getVertexCount() const {
        return m_vertexCount;
    }

    int getIndexCount() const {
        return m_indexCount;
    }

};

}; // ode_utils


#endif

#ifndef __ODE_UTILS_STL_READER_H_
#define __ODE_UTILS_STL_READER_H_

#include <sstream>
#include "TriMeshReaderBase.h"

namespace ode_utils {

template<typename V_TYPE,typename IDX_TYPE>
class StlReader
    : public TriMeshReaderBase<V_TYPE,IDX_TYPE>
{
public:
    typedef V_TYPE vertex_type;
    typedef IDX_TYPE index_type;

private:
    static const int head_size = 80;
    struct st_vec3f {
        float x; float y; float z;
    };
    struct st_param3f {
        st_vec3f n; st_vec3f v1; st_vec3f v2; st_vec3f v3;
    };

protected:
    // member
    char          *m_header;
    std::ostringstream m_msg;
    std::string        m_msgStr;

public:
    // constructor
    StlReader( const char* file_name )
        : TriMeshReaderBase<V_TYPE,IDX_TYPE>(file_name)
        , m_header(NULL)
    {
        m_header = new char[head_size];

        this->m_complete = createMeshData( file_name );
        if (this->m_complete) {
            m_msg << "stl file read - completed.\n";
            m_msg << "file:" << file_name << "\n";
        } else {
            m_msg << "stl file read - non completed.\n";
        }
    }

    ~StlReader()
    {
        if (m_header) delete [] m_header;
        m_header = NULL;
        if (this->m_vertices) delete [] this->m_vertices;
        this->m_vertices = NULL;
        if (this->m_indices) delete [] this->m_indices;
        this->m_indices = NULL;
    }

    virtual const char* message() {
        m_msgStr = m_msg.str();
        return m_msgStr.c_str();
    }

protected:
    virtual bool createMeshData( const char* file_name )
    {
        FILE *fp = fopen( file_name, "rb" );
        if (fp == NULL) {
            m_msg << "StlReader : can not open file.\n";
            m_msg << "file : " << file_name << "\n";
            return false;
        }

        unsigned int   triNum;
        st_param3f param;
        short int blank;

        int fsz;
        fseek( fp, 0, SEEK_END );
        fsz = ftell( fp );
        fseek(fp, 0, SEEK_SET );

        // header
        fread( m_header, sizeof(char), head_size, fp );
        fread( &triNum, sizeof(unsigned int), 1, fp );

        this->m_vertices = new vertex_type[3*3*triNum];
        this->m_indices = new index_type[3*triNum];

        if (fsz < (int)(head_size+4+(sizeof(st_param3f)+2)*triNum-2)) {
            m_msg << "StlReader : error, " << file_name << "\n";
            m_msg << "This file don't support file size.\n";
            m_msg << "file size : " << fsz << "bytes, ";
            m_msg << "trace size=" << (0+4+(sizeof(st_param3f)+2)*triNum ) << " bytes\n";
            return false;
        }

        for (int i=0; i<(int)triNum; i++) {
            fread( &param, sizeof(st_param3f), 1, fp );
            fread( &blank, sizeof(short int), 1, fp );
            this->m_vertices[i*3*3+0] = (vertex_type)param.v1.x;
            this->m_vertices[i*3*3+1] = (vertex_type)param.v1.y;
            this->m_vertices[i*3*3+2] = (vertex_type)param.v1.z;
            this->m_vertices[i*3*3+3] = (vertex_type)param.v2.x;
            this->m_vertices[i*3*3+4] = (vertex_type)param.v2.y;
            this->m_vertices[i*3*3+5] = (vertex_type)param.v2.z;
            this->m_vertices[i*3*3+6] = (vertex_type)param.v3.x;
            this->m_vertices[i*3*3+7] = (vertex_type)param.v3.y;
            this->m_vertices[i*3*3+8] = (vertex_type)param.v3.z;
        }

        for (int i=0; i<(int)triNum*3; i++) {
            this->m_indices[i] = i;
        }

        this->m_vertexCount = triNum* 3;
        this->m_indexCount = triNum * 3;

        fclose( fp );

        return true;
    }
};

}; // namespace ode_utils;


#endif

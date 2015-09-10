#ifndef __ODE_UTILS_PS_PIPESTREAM_H_
#define __ODE_UTILS_PS_PIPESTREAM_H_

#include <cstdio>
#include <iostream>
#include <sstream>

namespace ode_utils
{
namespace ps
{
struct __pendl {char dummy;} endl;
struct __pflush {char dummy;} flush;

class pipestream
{
private:
    FILE  *p;
public:
    pipestream (void) : p(NULL) {};
    pipestream (const char *name) : p(NULL) { open(name); };
    ~pipestream (void) { close(); }
    bool open (const char *name) {
        if (is_open())  close();
#ifdef WIN32
        p = _popen(name, "w");
#else
        p = popen(name, "w");
#endif
        if (p == NULL) {
            std::cerr<<"failed to open "<<name<<std::endl;
            return false;
        }
        return true;
    };
    void close (void) {
        if(is_open())  {
#ifdef WIN32
            _pclose(p);
#else
            pclose(p);
#endif
            p=NULL;
        }
    };
    bool is_open (void) const { return p!=NULL; };
    template <typename T>
    pipestream& operator<< (const T &rhs) {
        std::stringstream ss;
        ss<<rhs;
        fputs(ss.str().c_str(), p);
        return *this;
    };
    pipestream& operator<< (const char *rhs) {
        fputs(rhs, p);
        return *this;
    };
    pipestream& operator<< (const __pendl &rhs) {
        fputs("\n", p);
        fflush(p);
        return *this;
    };
    pipestream& operator<< (const __pflush &rhs) {
        fflush(p);
        return *this;
    };
};

}  // end of namespace ps
}  // end of namespace ode_utils

#endif // __ODE_UTILS_PS_PIPESTREAM_H_
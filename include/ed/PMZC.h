#ifndef ED_PMZC_H_
#define ED_PMZC_H_

namespace ed
{

class PMZC
{

public:
    PMZC(float min, float max, bool include) : min(min), max(max), include(include) {}

    float min;
    float max;
    bool include;

};

} // end namespace ed

#endif

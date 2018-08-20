#ifndef ED_ROI_H_
#define ED_ROI_H_

namespace ed
{

class ROI
{

public:
    ROI(float min, float max, bool include) : min(min), max(max), include(include) {}

    float min;
    float max;
    bool include;

};

} // end namespace ed

#endif

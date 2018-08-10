#ifndef ED_STATEDEFINITION_H_
#define ED_STATEDEFINITION_H_

namespace ed
{

class StateDefinition
{

public:
    StateDefinition(bool angle, bool position, float angleDifferenceClose, float angleDifferenceOpen,
      float positionDifferenceClose, float positionDifferenceOpen) :
      angle(angle), position(position), angleDifferenceClose(angleDifferenceClose),
      angleDifferenceOpen(angleDifferenceOpen), positionDifferenceClose(positionDifferenceClose),
      positionDifferenceOpen(positionDifferenceOpen) {}
    bool angle;
    bool position;
    float angleDifferenceClose;
    float angleDifferenceOpen;
    float positionDifferenceClose;
    float positionDifferenceOpen;
};

} // end namespace ed

#endif

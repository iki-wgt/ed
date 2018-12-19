#ifndef ED_MOVE_RESTRICTIONS_H
#define ED_MOVE_RESTRICTIONS_H

#include <geolib/datatypes.h>

namespace ed
{

class MoveRestrictions
{

public:
    MoveRestrictions(bool canRotate, bool canMove, float x, float y) :
        canRotate(canRotate), canMove(canMove), moveDirection(geo::Vec2(x, y)) {}
    bool canRotate;
    bool canMove;
    geo::Vec2 moveDirection;
};

} // end namespace ed

#endif

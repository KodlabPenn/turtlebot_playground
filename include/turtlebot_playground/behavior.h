/**
 * Sample behavior class
 * */

#include "turtlebot_playground/controller.h"
#include <Eigen/Dense>

class Behavior : public Controller
{

public:
    /** convenience type aliases **/
    using Vec2d = Eigen::Vector2d;
    using Vec3d = Eigen::Vector3d;

    Behavior();

    ~Behavior();

    /**
     * @brief This is your controller update loop where the behavior is 'updated'.
     * You are required to implement this function to inherit from the parent Controller class.
     * 
     */
    void Update() {
        twist << 0.25, 0.5;
        SetVelocity(twist);
    }

private:
    Vec2d twist;
};
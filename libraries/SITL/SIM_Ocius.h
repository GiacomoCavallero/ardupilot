/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  rover simulator class
*/

#pragma once

#include "SIM_Rover.h"

namespace SITL {

/*
  a rover simulator
 */
class SimOcius : public SimRover {
public:
	SimOcius(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override ;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new SimOcius(frame_str);
    }

    class StepMotor {
    public:
        StepMotor(float min_pos = -M_PI, float max_pos = M_PI, float max_velocity = 1, float initial_position = 0):
            position(initial_position), target(initial_position),
            min_position(min_pos), max_position(max_pos),
            velocity(max_velocity)
        {}

        void update(float interval) {
            if (fabsf((float)(target - position)) < FLT_EPSILON)
                return;
            float error = position - target;
            float delta = velocity * interval;

            if (fabs(error) < delta) {
                position = target;
            } else if (target < position) {
                position -= delta;
            } else {
                position += delta;
            }

            if (position < min_position)
                position = min_position;
            else if (position > max_position)
                position = max_position;
        }

        void setTarget(float new_target) {
            target = new_target;
        }

        // 0..1
        void setRelativeTarget(float new_target) {
            target = min_position + (new_target) * (max_position - min_position);
        }

        // 0..1
        float getRelativePosition() const {
            return (position - min_position) / (max_position - min_position);
        }

    protected:
        float position, target, min_position, max_position, velocity;

        friend class SimOcius;
    };
private:
    StepMotor rudder, mast, wing_sail, winch;
    bool is_wamv;
public:
    void getRelMotorPositions(float& rudderF, float& mastF, float& sailF, float& winchF) const {
        rudderF = rudder.getRelativePosition();
        mastF = mast.getRelativePosition();
        sailF = wing_sail.getRelativePosition();
        winchF = winch.getRelativePosition();
    }
};

} // namespace SITL

#ifndef MOVE_STATE_H
#define MOVE_STATE_H

#include "state.h"

/**
 * \brief Serves as the base for explore and travel states.
 */
class MoveState : public State {
private:
    const double position_threshold;
    const double velocity_threshold;
    const double speed;
    bool been_to_all_points = false;

protected:
    bool update_setpoint = false;
    std::vector<geometry_msgs::Point>::iterator current_destination_point_iterator;
    explicit MoveState(StateIdentifier state_identifier,
                       double speed,
                       double position_threshold,
                       double velocity_threshold) : State(state_identifier, PX4StateIdentifier::Offboard, false, true),
                                                    speed(speed),
                                                    position_threshold(position_threshold),
                                                    velocity_threshold(velocity_threshold) {}

public:
    bool hasFinishedExecution() const override;
    virtual void tick() override;
    virtual void initialize() override;
};

#endif

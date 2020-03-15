#ifndef CORE_H
#define CORE_H

#include "status_publisher.h"

#include <map>
#include <memory>

#include <geometry_msgs/Point32.h>

class Core {
   private:
    // Constructors are private as this class is used as a singleton with members.

    Core(){};
    Core(Core const&){};
    Core& operator=(Core const&){};

    static std::shared_ptr<StatusPublisher> status_publisher_ptr;

   public:
    static int refresh_rate;  ///< The unified refresh rate across
                              ///< the state machine.
                              ///< Specifies how fast the operation
                              ///< server and its underlying servies
                              ///< will run.

    static bool auto_arm;
    static bool auto_set_offboard;

    static double distance_completion_threshold;  ///< Specifies the radius for within we
                                                  ///< we can say that we are at the given position.

    static double velocity_completion_threshold;  ///< Specifies how low the velocity has to
                                                  ///< be before we issue that a given state
                                                  ///< is completed. This serves the purpose to prevent
                                                  ///< oscillations in the case when new operations are
                                                  ///< fired the moment the previous completes.
                                                  ///< With this threshold, we have to wait for the
                                                  ///< drone to be steady at the current position before
                                                  ///< carrying on.

    static double yaw_completion_threshold;  ///< Specifies the threshold for tolerance
                                             ///< of yaw, so the drone's yaw has to be within
                                             ///< this threshold before we continue doing another
                                             ///< operation.

    static double default_height;

    static std::shared_ptr<StatusPublisher> getStatusPublisherPtr();
};

#endif
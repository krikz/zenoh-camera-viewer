// idl.ts - IDL definitions for ROS messages

import { parseIDL } from "@lichtblick/omgidl-parser";

// Navigate To Pose IDL definition
const NAVIGATE_TO_POSE_IDL = `
  module builtin_interfaces {
    struct Time {
      uint32 sec;
      uint32 nanosec;
    };
  };

  module std_msgs {
    struct Header {
      builtin_interfaces::Time stamp;
      string frame_id;
    };
  };

  module geometry_msgs {
    struct Point {
      double x;
      double y;
      double z;
    };

    struct Quaternion {
      double x;
      double y;
      double z;
      double w;
    };

    struct Pose {
      geometry_msgs::Point position;
      geometry_msgs::Quaternion orientation;
    };

    struct PoseStamped {
      std_msgs::Header header;
      geometry_msgs::Pose pose;
    };
  };

  module nav2_msgs {
    struct NavigateToPose_Goal {
      geometry_msgs::PoseStamped pose;
      string behavior_tree;
    };
  };
`;

// Twist IDL definition
const TWIST_IDL = `
  module geometry_msgs {
    struct Vector3 {
      double x;
      double y;
      double z;
    };

    struct Twist {
      geometry_msgs::Vector3 linear;
      geometry_msgs::Vector3 angular;
    };
  };
`;

// Parse IDL definitions
export const navigateToPoseDefinition = parseIDL(NAVIGATE_TO_POSE_IDL);
export const twistDefinition = parseIDL(TWIST_IDL);

// Export types
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Twist {
  linear: Vector3;
  angular: Vector3;
}

export interface NavigateToPoseGoal {
  pose: {
    header: {
      stamp: {
        sec: number;
        nanosec: number;
      };
      frame_id: string;
    };
    pose: {
      position: {
        x: number;
        y: number;
        z: number;
      };
      orientation: {
        x: number;
        y: number;
        z: number;
        w: number;
      };
    };
  };
  behavior_tree: string;
}

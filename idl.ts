// idl.ts

import { parseIDL } from "@lichtblick/omgidl-parser";

// Полное определение сообщений, используемых в NavigateToPose
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

    struct Vector3 {
      double x;
      double y;
      double z;
    };

    struct Pose {
      Point position;
      Quaternion orientation;
    };

    struct PoseStamped {
      std_msgs::Header header;
      Pose pose;
    };
  };

  module nav2_msgs {
    struct NavigateToPose_Goal {
      geometry_msgs::PoseStamped pose;
      string behavior_tree;
    };
  };
`;

export const navigateToPoseDefinition = parseIDL(NAVIGATE_TO_POSE_IDL);
export type NavigateToPoseGoal = {
  header: {
    stamp: { sec: number; nanosec: number };
    frame_id: string;
  };
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  behavior_tree: string;
};

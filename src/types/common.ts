/**
 * TypeScript типы для базовых ROS структур
 */

export interface Time {
  sec: number;
  nanosec: number;
}

export interface Header {
  stamp: Time;
  frame_id: string;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Pose {
  position: Point;
  orientation: Quaternion;
}

export interface PoseStamped {
  header: Header;
  pose: Pose;
}

export interface Transform {
  translation: Vector3;
  rotation: Quaternion;
}

export interface TransformStamped {
  header: Header;
  child_frame_id: string;
  transform: Transform;
}

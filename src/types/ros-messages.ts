/**
 * TypeScript типы для ROS сообщений
 */

import { Header, Pose, Transform } from './common';

/**
 * sensor_msgs/Image
 */
export interface Image {
  header: Header;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: number[];
}

/**
 * sensor_msgs/CompressedImage
 */
export interface CompressedImage {
  header: Header;
  format: string;
  data: number[];
}

/**
 * sensor_msgs/LaserScan
 */
export interface LaserScan {
  header: Header;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

/**
 * nav_msgs/MapMetaData
 */
export interface MapMetaData {
  map_load_time: { sec: number; nanosec: number };
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
}

/**
 * nav_msgs/OccupancyGrid
 */
export interface OccupancyGrid {
  header: Header;
  info: MapMetaData;
  data: number[]; // int8[], -1 = unknown, 0 = free, 100 = occupied
}

/**
 * nav_msgs/Odometry
 */
export interface Odometry {
  header: Header;
  child_frame_id: string;
  pose: {
    pose: Pose;
  };
}

/**
 * tf2_msgs/TFMessage
 */
export interface TFMessage {
  transforms: Array<{
    header: Header;
    child_frame_id: string;
    transform: Transform;
  }>;
}

/**
 * nav_msgs/Path
 */
export interface Path {
  header: Header;
  poses: Array<{
    pose: Pose;
  }>;
}

/**
 * geometry_msgs/Twist
 */
export interface Twist {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

/**
 * Валидаторы для ROS сообщений
 */

import type {
  OccupancyGrid,
  Image,
  LaserScan,
  Odometry,
  Path,
  TFMessage,
} from '../types';

/**
 * Проверяет корректность OccupancyGrid
 */
export function isValidOccupancyGrid(msg: any): msg is OccupancyGrid {
  return (
    msg &&
    msg.info &&
    typeof msg.info.width === 'number' &&
    typeof msg.info.height === 'number' &&
    typeof msg.info.resolution === 'number' &&
    Array.isArray(msg.data) &&
    msg.data.length === msg.info.width * msg.info.height
  );
}

/**
 * Проверяет корректность Image
 */
export function isValidImage(msg: any): msg is Image {
  return (
    msg &&
    typeof msg.width === 'number' &&
    typeof msg.height === 'number' &&
    typeof msg.encoding === 'string' &&
    typeof msg.step === 'number' &&
    Array.isArray(msg.data) &&
    msg.data.length > 0
  );
}

/**
 * Проверяет корректность LaserScan
 */
export function isValidLaserScan(msg: any): msg is LaserScan {
  return (
    msg &&
    typeof msg.angle_min === 'number' &&
    typeof msg.angle_max === 'number' &&
    typeof msg.angle_increment === 'number' &&
    Array.isArray(msg.ranges) &&
    msg.ranges.length > 0
  );
}

/**
 * Проверяет корректность Odometry
 */
export function isValidOdometry(msg: any): msg is Odometry {
  return (
    msg &&
    msg.pose &&
    msg.pose.pose &&
    msg.pose.pose.position &&
    typeof msg.pose.pose.position.x === 'number' &&
    typeof msg.pose.pose.position.y === 'number' &&
    msg.pose.pose.orientation &&
    typeof msg.pose.pose.orientation.w === 'number'
  );
}

/**
 * Проверяет корректность Path
 */
export function isValidPath(msg: any): msg is Path {
  return (
    msg &&
    Array.isArray(msg.poses) &&
    msg.poses.every(
      (p: any) =>
        p.pose &&
        p.pose.position &&
        typeof p.pose.position.x === 'number' &&
        typeof p.pose.position.y === 'number'
    )
  );
}

/**
 * Проверяет корректность TFMessage
 */
export function isValidTFMessage(msg: any): msg is TFMessage {
  return (
    msg &&
    Array.isArray(msg.transforms) &&
    msg.transforms.every(
      (t: any) =>
        t.header &&
        typeof t.header.frame_id === 'string' &&
        typeof t.child_frame_id === 'string' &&
        t.transform
    )
  );
}

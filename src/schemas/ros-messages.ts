/**
 * CDR схемы для всех ROS сообщений
 * Автоматически генерируются с помощью schema generator
 */

import {
  dictionary,
  field,
  sequence,
  baseTypes,
  headerSchema,
  poseSchema,
  transformSchema,
} from './generator';

/**
 * sensor_msgs/Image
 */
export const imageSchema = dictionary({
  header: field(0, headerSchema),
  height: field(1, baseTypes.uint32),
  width: field(2, baseTypes.uint32),
  encoding: field(3, baseTypes.string),
  is_bigendian: field(4, baseTypes.int8),
  step: field(5, baseTypes.uint32),
  data: field(6, sequence(baseTypes.uint8)),
});

/**
 * sensor_msgs/CompressedImage
 */
export const compressedImageSchema = dictionary({
  header: field(0, headerSchema),
  format: field(1, baseTypes.string),
  data: field(2, sequence(baseTypes.uint8)),
});

/**
 * sensor_msgs/LaserScan
 */
export const laserScanSchema = dictionary({
  header: field(0, headerSchema),
  angle_min: field(1, baseTypes.float32),
  angle_max: field(2, baseTypes.float32),
  angle_increment: field(3, baseTypes.float32),
  time_increment: field(4, baseTypes.float32),
  scan_time: field(5, baseTypes.float32),
  range_min: field(6, baseTypes.float32),
  range_max: field(7, baseTypes.float32),
  ranges: field(8, sequence(baseTypes.float32)),
  intensities: field(9, sequence(baseTypes.float32)),
});

/**
 * nav_msgs/MapMetaData
 */
const mapMetaDataSchema = dictionary({
  map_load_time: field(0, dictionary({
    sec: field(0, baseTypes.uint32),
    nanosec: field(1, baseTypes.uint32),
  })),
  resolution: field(1, baseTypes.float32),
  width: field(2, baseTypes.uint32),
  height: field(3, baseTypes.uint32),
  origin: field(4, poseSchema),
});

/**
 * nav_msgs/OccupancyGrid
 */
export const occupancyGridSchema = dictionary({
  header: field(0, headerSchema),
  info: field(1, mapMetaDataSchema),
  data: field(2, sequence(baseTypes.int8)),
});

/**
 * geometry_msgs/PoseWithCovariance
 */
const poseWithCovarianceSchema = dictionary({
  pose: field(0, poseSchema),
  // covariance пропускаем для упрощения
});

/**
 * nav_msgs/Odometry
 */
export const odometrySchema = dictionary({
  header: field(0, headerSchema),
  child_frame_id: field(1, baseTypes.string),
  pose: field(2, poseWithCovarianceSchema),
  // twist пропускаем для упрощения
});

/**
 * geometry_msgs/TransformStamped
 */
const transformStampedSchema = dictionary({
  header: field(0, headerSchema),
  child_frame_id: field(1, baseTypes.string),
  transform: field(2, transformSchema),
});

/**
 * tf2_msgs/TFMessage
 */
export const tfMessageSchema = dictionary({
  transforms: field(0, sequence(transformStampedSchema)),
});

/**
 * geometry_msgs/PoseStamped (для Path)
 */
const poseStampedSchema = dictionary({
  pose: field(0, poseSchema),
  // header пропускаем в item для упрощения
});

/**
 * nav_msgs/Path
 */
export const pathSchema = dictionary({
  header: field(0, headerSchema),
  poses: field(1, sequence(poseStampedSchema)),
});

/**
 * rcl_interfaces/Log
 */
export const logSchema = dictionary({
  timestamp: field(0, dictionary({
    sec: field(0, baseTypes.uint32),
    nanosec: field(1, baseTypes.uint32),
  })),
  level: field(1, baseTypes.uint8),
  name: field(2, baseTypes.string),
  msg: field(3, baseTypes.string),
  file: field(4, baseTypes.string),
  function: field(5, baseTypes.string),
  line: field(6, baseTypes.uint32),
});

/**
 * geometry_msgs/Twist
 */
import { vector3Schema } from './generator';

export const twistSchema = dictionary({
  linear: field(0, vector3Schema),
  angular: field(1, vector3Schema),
});

/**
 * Типы для конфигурации роботов
 */

export interface CameraConfig {
  id: string;
  name: string;
  topic: string;
  encoding: 'compressed' | 'raw' | 'bgr8' | 'rgb8' | 'mono8';
  default?: boolean;
}

export interface RobotTopics {
  map: string;
  lidar: string;
  odometry: string;
  tf: string;
  plan: string;
  cmd_vel: string;
}

export interface RobotConfig {
  id: string;
  name: string;
  description?: string;
  cameras: CameraConfig[];
  topics: RobotTopics;
}

export interface RobotsConfiguration {
  robots: RobotConfig[];
}

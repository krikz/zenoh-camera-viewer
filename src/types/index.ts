/**
 * Экспорт всех типов
 */

export * from './common';
export * from './ros-messages';

/**
 * Внутренние типы приложения
 */

export interface RobotPosition {
  x: number;
  y: number;
  theta: number;
}

export interface Waypoint {
  x: number;
  y: number;
}

export interface CameraState {
  x: number;
  y: number;
  width: number;
  height: number;
}

export enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  ERROR = 'error',
}

/**
 * Конфигурация для EventSource фида
 */
export interface FeedConfig {
  robotName: string;
  topic: string;
  maxSequenceSize?: number;
}

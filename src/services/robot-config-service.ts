/**
 * Сервис для работы с конфигурацией роботов
 */

import type { RobotsConfiguration, RobotConfig, CameraConfig } from '../types';
import robotsData from '../config/robots.json';

class RobotConfigService {
  private config: RobotsConfiguration;

  constructor() {
    this.config = robotsData as RobotsConfiguration;
  }

  /**
   * Получает список всех роботов
   */
  getRobots(): RobotConfig[] {
    return this.config.robots;
  }

  /**
   * Получает конфигурацию робота по ID
   */
  getRobotById(id: string): RobotConfig | undefined {
    return this.config.robots.find(robot => robot.id === id);
  }

  /**
   * Получает список камер для робота
   */
  getCamerasForRobot(robotId: string): CameraConfig[] {
    const robot = this.getRobotById(robotId);
    return robot?.cameras || [];
  }

  /**
   * Получает камеру по умолчанию для робота
   */
  getDefaultCamera(robotId: string): CameraConfig | undefined {
    const cameras = this.getCamerasForRobot(robotId);
    return cameras.find(cam => cam.default) || cameras[0];
  }

  /**
   * Получает топики для робота
   */
  getTopicsForRobot(robotId: string) {
    const robot = this.getRobotById(robotId);
    return robot?.topics;
  }

  /**
   * Проверяет, существует ли робот в конфигурации
   */
  hasRobot(robotId: string): boolean {
    return this.config.robots.some(robot => robot.id === robotId);
  }
}

// Экспортируем синглтон
export const robotConfigService = new RobotConfigService();

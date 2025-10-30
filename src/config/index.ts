/**
 * Конфигурация приложения
 * Все настройки и константы в одном месте
 */

/**
 * Настройки Zenoh
 */
export const ZENOH_CONFIG = {
  /**
   * Базовый URL для Zenoh REST API
   * Можно переопределить через переменную окружения
   */
  REST_BASE: import.meta.env.VITE_ZENOH_URL || 'https://zenoh.robbox.online',
  
  /**
   * Домен ROS (обычно 0)
   * Используется в путях: robots/{robot}/{domain}/{topic}
   */
  ROS_DOMAIN: '0',
  
  /**
   * Задержка перед попыткой переподключения (мс)
   */
  RECONNECT_DELAY: 5000,
  
  /**
   * Максимальное количество попыток переподключения
   */
  MAX_RECONNECT_ATTEMPTS: 5,
} as const;

/**
 * Размеры для парсинга CDR сообщений
 */
export const CDR_LIMITS = {
  IMAGE: 300_000,
  MAP: 1_000_000,
  LIDAR: 10_000,
  ODOMETRY: 10_000,
  TF: 10_000,
  PATH: 10_000,
  DEFAULT: 10_000,
} as const;

/**
 * Настройки Canvas
 */
export const CANVAS_CONFIG = {
  MAP: {
    WIDTH: 600,
    HEIGHT: 600,
  },
  LIDAR: {
    WIDTH: 600,
    HEIGHT: 600,
  },
  CAMERA: {
    DEFAULT_WIDTH: 400,
    DEFAULT_HEIGHT: 300,
  },
} as const;

/**
 * Настройки отрисовки
 */
export const RENDER_CONFIG = {
  /**
   * Цвета для карты
   */
  MAP_COLORS: {
    UNKNOWN: 'gray',
    FREE: 'white',
    OCCUPIED: 'black',
  },
  
  /**
   * Цвета для лидара
   */
  LIDAR_COLORS: {
    POINT: 'red',
    ROBOT: 'blue',
    TRAJECTORY: 'green',
    WAYPOINT: 'orange',
  },
  
  /**
   * Размеры элементов
   */
  SIZES: {
    ROBOT_RADIUS: 5,
    LIDAR_POINT_SIZE: 2,
    WAYPOINT_RADIUS: 4,
    TRAJECTORY_WIDTH: 2,
  },
} as const;

/**
 * Настройки геймпада
 */
export const GAMEPAD_CONFIG = {
  /**
   * Частота опроса геймпада (мс)
   */
  POLL_INTERVAL: 50,
  
  /**
   * Мертвая зона для стиков (0.0 - 1.0)
   */
  DEADZONE: 0.1,
  
  /**
   * Максимальные скорости
   */
  MAX_LINEAR_SPEED: 0.5,      // м/с
  MAX_ANGULAR_SPEED: 1.0,     // рад/с
  
  /**
   * Маппинг осей геймпада
   * Стандартные значения для Xbox/PS контроллеров:
   * - PITCH_AXIS: 1 (левый стик Y - вперед/назад)
   * - YAW_AXIS: 3 (правый стик X - поворот)
   * 
   * Для FrSky Taranis X7 (режим Mode 2):
   * - PITCH_AXIS: 1 (правый стик Y - Elevator/Pitch)
   * - YAW_AXIS: 3 (левый стик X - Rudder/Yaw)
   * 
   * ⚠️ Если управление работает неправильно, откройте gamepad-test.html
   * и проверьте какие оси активны при движении стиков!
   */
  PITCH_AXIS: 1,              // Ось для движения вперед/назад
  YAW_AXIS: 3,                // Ось для поворота
} as const;

/**
 * Ключи для localStorage
 */
export const STORAGE_KEYS = {
  CAMERA_POSITION: 'cameraPosition',
  SELECTED_ROBOT: 'selectedRobot',
  WAYPOINTS: 'waypoints',
} as const;

/**
 * Топики ROS для разных фидов
 */
export const ROS_TOPICS = {
  CAMERA: 'robot_cam',
  MAP: 'map',
  LIDAR: 'scan',
  ODOMETRY: 'diff_drive_base_controller/odom',
  TF: 'tf',
  PLAN: 'plan',
  CMD_VEL: 'cmd_vel',
  NAVIGATE_TO_POSE: 'navigate_to_pose/_action/send_goal',
} as const;

/**
 * Режимы навигации
 */
export enum NavigationMode {
  SINGLE_GOAL = 'single',
  WAYPOINTS = 'waypoints',
  EXPLORE = 'explore',
  SNAKE = 'snake',
}

/**
 * Логирование
 */
export const LOG_CONFIG = {
  /**
   * Включить отладочные сообщения
   */
  DEBUG: import.meta.env.DEV,
  
  /**
   * Префиксы для разных модулей
   */
  PREFIXES: {
    ZENOH: '[Zenoh]',
    RENDERER: '[Renderer]',
    PARSER: '[Parser]',
    UI: '[UI]',
    GAMEPAD: '[Gamepad]',
  },
} as const;

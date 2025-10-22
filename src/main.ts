/**
 * Главная точка входа приложения
 * Zenoh Robot Explorer - современная модульная версия
 */

import { ZenohClient } from './services/zenoh-client';
import { MapRenderer, CameraRenderer, LidarRenderer } from './renderers';
import { CameraController, GamepadController } from './ui';
import { parseZenohMessage, logger, isValidImage, isValidOccupancyGrid, isValidLaserScan, isValidOdometry, isValidPath, isValidTFMessage, transformOdomToMap } from './utils';
import { imageSchema, laserScanSchema, occupancyGridSchema, odometrySchema, pathSchema, tfMessageSchema } from './schemas';
import type { Image, LaserScan, OccupancyGrid, Odometry, Path, TFMessage, RobotPosition, Transform } from './types';
import { ZENOH_CONFIG, CDR_LIMITS, ROS_TOPICS, LOG_CONFIG } from './config';

// ==================== DOM Elements ====================
const robotSelect = document.getElementById('robotSelect') as HTMLSelectElement;
const statusEl = document.getElementById('status') as HTMLElement;
const mapCanvas = document.getElementById('mapCanvas') as HTMLCanvasElement;
const lidarCanvas = document.getElementById('lidarCanvas') as HTMLCanvasElement;
const cameraCanvas = document.getElementById('cameraCanvas') as HTMLCanvasElement;

// UI элементы для контроллеров
const cameraOverlay = document.querySelector('.camera-overlay') as HTMLElement;
const resizeHandle = cameraOverlay?.querySelector('.resize-handle') as HTMLElement;
const gamepadBtn = document.getElementById('gamepadBtn') as HTMLButtonElement;
const gamepadOverlay = document.querySelector('.gamepad-overlay') as HTMLElement;
const pitchIndicator = document.getElementById('pitchIndicator') as HTMLElement;
const yawIndicator = document.getElementById('yawIndicator') as HTMLElement;
const robotVisual = document.querySelector('.robot-visual') as HTMLElement;
const robotCanvas = document.getElementById('robotCanvas') as HTMLCanvasElement;

// ==================== Renderers ====================
const mapRenderer = new MapRenderer(mapCanvas);
const lidarRenderer = new LidarRenderer(lidarCanvas);
const cameraRenderer = new CameraRenderer(cameraCanvas);

// ==================== Services ====================
const zenohClient = new ZenohClient();

// ==================== UI Controllers ====================
let cameraController: CameraController | null = null;
let gamepadController: GamepadController | null = null;

// Инициализация UI контроллеров после загрузки DOM
if (cameraOverlay && resizeHandle) {
  cameraController = new CameraController(cameraOverlay, cameraCanvas, resizeHandle);
}

if (gamepadBtn && gamepadOverlay && pitchIndicator && yawIndicator && robotVisual && robotCanvas) {
  gamepadController = new GamepadController(
    gamepadBtn,
    gamepadOverlay,
    pitchIndicator,
    yawIndicator,
    robotVisual,
    robotCanvas
  );
  
  // Устанавливаем callback для публикации команд
  gamepadController.setPublishCallback(async (_topic: string, _data: Uint8Array) => {
    // TODO: Реализовать публикацию в Zenoh
    // await zenohClient.publish(currentRobotName, topic, data);
  });
}

// ==================== State ====================
let currentRobotName = '';
let robotPosition: RobotPosition = { x: 0, y: 0, theta: 0 };
let currentPlan: Array<{ x: number; y: number }> = [];
let mapToOdom: Transform = {
  translation: { x: 0, y: 0, z: 0 },
  rotation: { x: 0, y: 0, z: 0, w: 1 },
};

// ==================== Message Handlers ====================

function handleCameraMessage(data: string): void {
  const image = parseZenohMessage<Image>(data, imageSchema, CDR_LIMITS.IMAGE);
  
  if (!image || !isValidImage(image)) {
    logger.error(LOG_CONFIG.PREFIXES.RENDERER, 'Некорректные данные камеры');
    return;
  }

  statusEl.textContent = `🎥 ${image.width}x${image.height}, ${image.encoding}`;
  cameraRenderer.render(image);
}

function handleMapMessage(data: string): void {
  const map = parseZenohMessage<OccupancyGrid>(data, occupancyGridSchema, CDR_LIMITS.MAP);
  
  if (!map || !isValidOccupancyGrid(map)) {
    logger.error(LOG_CONFIG.PREFIXES.RENDERER, 'Некорректные данные карты');
    return;
  }

  mapRenderer.renderMap(map);
  
  // Перерисовываем позицию робота и траекторию
  if (robotPosition) {
    mapRenderer.renderRobotPosition(robotPosition);
  }
  if (currentPlan.length > 0) {
    mapRenderer.renderTrajectory(currentPlan);
  }
}

function handleLidarMessage(data: string): void {
  const scan = parseZenohMessage<LaserScan>(data, laserScanSchema, CDR_LIMITS.LIDAR);
  
  if (!scan || !isValidLaserScan(scan)) {
    logger.error(LOG_CONFIG.PREFIXES.RENDERER, 'Некорректные данные лидара');
    return;
  }

  lidarRenderer.render(scan, robotPosition);
}

function handleOdometryMessage(data: string): void {
  const odom = parseZenohMessage<Odometry>(data, odometrySchema, CDR_LIMITS.ODOMETRY);
  
  if (!odom || !isValidOdometry(odom)) {
    logger.error(LOG_CONFIG.PREFIXES.PARSER, 'Некорректные данные одометрии');
    return;
  }

  const odomX = odom.pose.pose.position.x;
  const odomY = odom.pose.pose.position.y;
  const q = odom.pose.pose.orientation;

  // Преобразуем кватернион в угол
  const odomTheta = Math.atan2(
    2 * (q.w * q.z + q.x * q.y),
    1 - 2 * (q.y * q.y + q.z * q.z)
  );

  // Преобразуем из odom в map
  robotPosition = transformOdomToMap(odomX, odomY, odomTheta, mapToOdom);

  // Обновляем отображение на карте
  const currentMap = mapRenderer.getCurrentMap();
  if (currentMap) {
    mapRenderer.renderMap(currentMap);
    mapRenderer.renderRobotPosition(robotPosition);
    if (currentPlan.length > 0) {
      mapRenderer.renderTrajectory(currentPlan);
    }
  }
}

function handleTfMessage(data: string): void {
  const tfMsg = parseZenohMessage<TFMessage>(data, tfMessageSchema, CDR_LIMITS.TF);
  
  if (!tfMsg || !isValidTFMessage(tfMsg)) {
    return;
  }

  // Ищем трансформацию от map к odom
  for (const transform of tfMsg.transforms) {
    if (transform.header.frame_id === 'map' && transform.child_frame_id === 'odom') {
      mapToOdom = transform.transform;
      break;
    }
  }
}

function handlePlanMessage(data: string): void {
  const path = parseZenohMessage<Path>(data, pathSchema, CDR_LIMITS.PATH);
  
  if (!path || !isValidPath(path)) {
    return;
  }

  currentPlan = path.poses.map((pose) => ({
    x: pose.pose.position.x,
    y: pose.pose.position.y,
  }));

  // Перерисовываем карту с траекторией
  const currentMap = mapRenderer.getCurrentMap();
  if (currentMap) {
    mapRenderer.renderMap(currentMap);
    mapRenderer.renderRobotPosition(robotPosition);
    mapRenderer.renderTrajectory(currentPlan);
  }
}

// ==================== Robot Management ====================

async function loadRobots(): Promise<void> {
  try {
    statusEl.textContent = '🔍 Поиск роботов...';
    const robots = await zenohClient.fetchRobots();

    robotSelect.innerHTML = '';
    
    if (robots.length === 0) {
      robotSelect.innerHTML = '<option>Роботы не найдены</option>';
      statusEl.textContent = '❌ Роботы не найдены';
      return;
    }

    robotSelect.innerHTML = '<option value="">-- Выберите робота --</option>';
    robots.forEach((name) => {
      const opt = document.createElement('option');
      opt.value = name;
      opt.textContent = name;
      robotSelect.appendChild(opt);
    });

    statusEl.textContent = `✅ Найдено роботов: ${robots.length}`;
  } catch (err) {
    logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка загрузки роботов:', err);
    robotSelect.innerHTML = '<option>Ошибка загрузки</option>';
    statusEl.textContent = '❌ Ошибка подключения';
  }
}

function connectToRobot(robotName: string): void {
  // Отключаемся от предыдущего робота
  if (currentRobotName) {
    zenohClient.unsubscribeRobot(currentRobotName);
    mapRenderer.clear();
    lidarRenderer.clear();
    cameraRenderer.clear();
  }

  currentRobotName = robotName;
  
  // Обновляем gamepad controller с новым роботом
  if (gamepadController) {
    gamepadController.setRobot(robotName);
  }
  
  statusEl.textContent = `📡 Подключение к ${robotName}...`;

  // Подписываемся на все топики
  zenohClient.subscribe(robotName, ROS_TOPICS.CAMERA, handleCameraMessage);
  zenohClient.subscribe(robotName, ROS_TOPICS.MAP, handleMapMessage);
  zenohClient.subscribe(robotName, ROS_TOPICS.LIDAR, handleLidarMessage);
  zenohClient.subscribe(robotName, ROS_TOPICS.ODOMETRY, handleOdometryMessage);
  zenohClient.subscribe(robotName, ROS_TOPICS.TF, handleTfMessage);
  zenohClient.subscribe(robotName, ROS_TOPICS.PLAN, handlePlanMessage);

  statusEl.textContent = `✅ Подключено к ${robotName}`;
  logger.info(LOG_CONFIG.PREFIXES.ZENOH, `Подключено к роботу ${robotName}`);
}

// ==================== Event Listeners ====================

robotSelect.addEventListener('change', () => {
  const robotName = robotSelect.value;
  if (robotName) {
    connectToRobot(robotName);
  } else {
    if (currentRobotName) {
      zenohClient.unsubscribeRobot(currentRobotName);
      currentRobotName = '';
    }
    statusEl.textContent = 'Выберите робота';
  }
});

// ==================== Initialization ====================

async function init(): Promise<void> {
  logger.info(LOG_CONFIG.PREFIXES.ZENOH, 'Инициализация приложения...');
  await loadRobots();
}

// Запуск приложения
init().catch((err) => {
  logger.error(LOG_CONFIG.PREFIXES.ZENOH, 'Ошибка инициализации:', err);
  statusEl.textContent = '❌ Ошибка инициализации';
});

// Очистка при закрытии страницы
window.addEventListener('beforeunload', () => {
  zenohClient.unsubscribeAll();
});

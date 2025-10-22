/**
 * Управление геймпадом для управления роботом
 */

import { GAMEPAD_CONFIG, LOG_CONFIG } from '../config';
import { logger } from '../utils/logger';
import type { Twist } from '../types';

export class GamepadController {
  private connected = false;
  private intervalId: number | null = null;
  private currentRobotName = '';
  private publishCallback: ((topic: string, data: Uint8Array) => Promise<void>) | null = null;

  constructor(
    private button: HTMLButtonElement,
    private overlay: HTMLElement,
    private pitchIndicator: HTMLElement,
    private yawIndicator: HTMLElement,
    private robotVisual: HTMLElement,
    private robotCanvas: HTMLCanvasElement
  ) {
    this.setupEventListeners();
  }

  /**
   * Устанавливает callback для публикации команд
   */
  setPublishCallback(callback: (topic: string, data: Uint8Array) => Promise<void>): void {
    this.publishCallback = callback;
  }

  /**
   * Устанавливает текущего робота
   */
  setRobot(robotName: string): void {
    this.currentRobotName = robotName;
  }

  /**
   * Настраивает обработчики событий
   */
  private setupEventListeners(): void {
    // Проверка поддержки Gamepad API
    if (!('getGamepads' in navigator)) {
      this.button.disabled = true;
      this.button.title = 'Браузер не поддерживает Gamepad API';
      this.button.textContent = '🎮 Gamepad недоступен';
      return;
    }

    // Событие подключения геймпада - АВТОМАТИЧЕСКОЕ ПОДКЛЮЧЕНИЕ
    window.addEventListener('gamepadconnected', (e: GamepadEvent) => {
      logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `Геймпад подключен: ${e.gamepad.id}`);
      if (!this.connected) {
        logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, 'Автоматическое подключение геймпада...');
        this.connect();
      }
    });

    // Событие отключения геймпада
    window.addEventListener('gamepaddisconnected', (e: GamepadEvent) => {
      logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `Геймпад отключен: ${e.gamepad.id}`);
      if (this.connected) {
        this.disconnect();
      }
    });

    // Кнопка подключения/отключения
    this.button.addEventListener('click', () => {
      if (this.connected) {
        this.disconnect();
      } else {
        this.connect();
      }
    });
    
    // Проверяем, может геймпад уже подключен (был подключен до загрузки страницы)
    this.checkExistingGamepads();
  }

  /**
   * Проверяет уже подключенные геймпады при инициализации
   */
  private checkExistingGamepads(): void {
    const gamepads = navigator.getGamepads();
    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        logger.info(
          LOG_CONFIG.PREFIXES.GAMEPAD,
          `Обнаружен подключенный геймпад: ${gamepads[i]!.id}`
        );
        // Обновляем UI чтобы показать, что геймпад доступен
        this.button.textContent = '🎮 Геймпад обнаружен - нажмите для активации';
        this.button.style.backgroundColor = '#4CAF50';
        break;
      }
    }
  }

  /**
   * Подключает геймпад
   */
  private connect(): void {
    const gamepads = navigator.getGamepads();
    let gamepad: Gamepad | null = null;

    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        gamepad = gamepads[i];
        break;
      }
    }

    if (!gamepad) {
      logger.warn(
        LOG_CONFIG.PREFIXES.GAMEPAD,
        '❌ Геймпад не найден. Подключите геймпад и нажмите любую кнопку для активации'
      );
      alert('Геймпад не найден!\n\n1. Подключите геймпад к компьютеру\n2. Нажмите любую кнопку на геймпаде\n3. Попробуйте снова');
      return;
    }

    // Запускаем опрос геймпада
    if (this.intervalId) {
      clearInterval(this.intervalId);
    }

    this.intervalId = window.setInterval(() => this.poll(), GAMEPAD_CONFIG.POLL_INTERVAL);
    this.connected = true;

    // Обновляем UI
    this.button.classList.add('connected');
    this.button.textContent = '⏹ Отключить геймпад';
    this.button.style.backgroundColor = '#f44336';
    this.overlay.style.display = 'block';
    this.robotVisual.style.display = 'block';

    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `✅ Геймпад активирован: ${gamepad.id}`);
    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `📊 Осей: ${gamepad.axes.length}, Кнопок: ${gamepad.buttons.length}`);
  }

  /**
   * Отключает геймпад
   */
  private disconnect(): void {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }

    // Отправляем команду остановки
    this.publishTwist(0, 0);

    // Обновляем UI
    this.connected = false;
    this.button.classList.remove('connected');
    this.button.textContent = '🎮 Connect Gamepad';
    this.overlay.style.display = 'none';
    this.robotVisual.style.display = 'none';

    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, 'Геймпад отключен');
  }

  /**
   * Опрашивает геймпад и отправляет команды
   */
  private poll(): void {
    const gamepads = navigator.getGamepads();
    if (gamepads.length === 0 || !gamepads[0]) {
      logger.warn(LOG_CONFIG.PREFIXES.GAMEPAD, 'Геймпад потерян, отключение...');
      this.disconnect();
      return;
    }

    const gamepad = gamepads[0];

    // Получаем значения осей из конфигурации
    const pitch = gamepad.axes[GAMEPAD_CONFIG.PITCH_AXIS]; // По умолчанию: ось 1
    const yaw = gamepad.axes[GAMEPAD_CONFIG.YAW_AXIS];     // По умолчанию: ось 3

    // Применяем мертвую зону
    const pitchValue = Math.abs(pitch) > GAMEPAD_CONFIG.DEADZONE ? pitch : 0;
    const yawValue = Math.abs(yaw) > GAMEPAD_CONFIG.DEADZONE ? yaw : 0;

    // Обновляем визуализацию
    this.updateVisualization(pitchValue, yawValue);

    // Если оба значения в мертвой зоне - останавливаем
    if (pitchValue === 0 && yawValue === 0) {
      this.publishTwist(0, 0);
      return;
    }

    // Преобразуем в линейную и угловую скорость
    const linear = pitchValue * GAMEPAD_CONFIG.MAX_LINEAR_SPEED;
    const angular = -yawValue * GAMEPAD_CONFIG.MAX_ANGULAR_SPEED;

    // Визуализация робота
    this.renderRobotVisualization(linear, angular);

    // Отправляем команду
    this.publishTwist(linear, angular);
  }

  /**
   * Обновляет визуализацию индикаторов
   */
  private updateVisualization(pitch: number, yaw: number): void {
    // Pitch: -1 (назад) -> 0%, 0 (нейтраль) -> 50%, 1 (вперед) -> 100%
    const pitchPercent = (pitch + 1) * 50;
    this.pitchIndicator.style.width = `${pitchPercent}%`;
    this.pitchIndicator.style.backgroundColor = pitch >= 0 ? '#4CAF50' : '#f44336';

    // Yaw: -1 (вправо) -> 0%, 0 (нейтраль) -> 50%, 1 (влево) -> 100%
    const yawPercent = (yaw + 1) * 50;
    this.yawIndicator.style.width = `${yawPercent}%`;
    this.yawIndicator.style.backgroundColor = '#2196F3';
  }

  /**
   * Отрисовывает визуализацию состояния робота
   */
  private renderRobotVisualization(linear: number, angular: number): void {
    const ctx = this.robotCanvas.getContext('2d');
    if (!ctx) return;

    // Очистка
    ctx.clearRect(0, 0, this.robotCanvas.width, this.robotCanvas.height);

    // Параметры робота
    const robotWidth = 60;
    const robotHeight = 80;
    const wheelWidth = 10;
    const wheelHeight = 20;

    const centerX = this.robotCanvas.width / 2;
    const centerY = this.robotCanvas.height / 2;

    // Корпус робота
    ctx.fillStyle = '#333';
    ctx.fillRect(
      centerX - robotWidth / 2,
      centerY - robotHeight / 2,
      robotWidth,
      robotHeight
    );

    // Колеса
    ctx.fillStyle = '#555';
    const wheelOffset = 15;
    
    // Левые колеса
    ctx.fillRect(
      centerX - robotWidth / 2 - 5,
      centerY - robotHeight / 2 + wheelOffset - wheelHeight / 2,
      wheelWidth,
      wheelHeight
    );
    ctx.fillRect(
      centerX - robotWidth / 2 - 5,
      centerY + robotHeight / 2 - wheelOffset - wheelHeight / 2,
      wheelWidth,
      wheelHeight
    );

    // Правые колеса
    ctx.fillRect(
      centerX + robotWidth / 2 - 5,
      centerY - robotHeight / 2 + wheelOffset - wheelHeight / 2,
      wheelWidth,
      wheelHeight
    );
    ctx.fillRect(
      centerX + robotWidth / 2 - 5,
      centerY + robotHeight / 2 - wheelOffset - wheelHeight / 2,
      wheelWidth,
      wheelHeight
    );

    // Вычисляем скорости колес
    const leftSpeed = linear - angular * 0.3;
    const rightSpeed = linear + angular * 0.3;

    // Нормализуем
    const normalizedLeft = Math.max(-1, Math.min(1, leftSpeed));
    const normalizedRight = Math.max(-1, Math.min(1, rightSpeed));

    // Векторы скорости
    const vectorLength = 40;

    // Левый вектор
    ctx.strokeStyle = normalizedLeft >= 0 ? '#4CAF50' : '#f44336';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX - 15, centerY);
    ctx.lineTo(centerX - 15, centerY - vectorLength * normalizedLeft);
    ctx.stroke();

    // Стрелка левого вектора
    const leftArrowY = centerY - vectorLength * normalizedLeft;
    ctx.fillStyle = ctx.strokeStyle;
    ctx.beginPath();
    if (normalizedLeft >= 0) {
      ctx.moveTo(centerX - 15, leftArrowY);
      ctx.lineTo(centerX - 19, leftArrowY + 8);
      ctx.lineTo(centerX - 11, leftArrowY + 8);
    } else {
      ctx.moveTo(centerX - 15, leftArrowY);
      ctx.lineTo(centerX - 19, leftArrowY - 8);
      ctx.lineTo(centerX - 11, leftArrowY - 8);
    }
    ctx.closePath();
    ctx.fill();

    // Правый вектор
    ctx.strokeStyle = normalizedRight >= 0 ? '#4CAF50' : '#f44336';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX + 15, centerY);
    ctx.lineTo(centerX + 15, centerY - vectorLength * normalizedRight);
    ctx.stroke();

    // Стрелка правого вектора
    const rightArrowY = centerY - vectorLength * normalizedRight;
    ctx.fillStyle = ctx.strokeStyle;
    ctx.beginPath();
    if (normalizedRight >= 0) {
      ctx.moveTo(centerX + 15, rightArrowY);
      ctx.lineTo(centerX + 11, rightArrowY + 8);
      ctx.lineTo(centerX + 19, rightArrowY + 8);
    } else {
      ctx.moveTo(centerX + 15, rightArrowY);
      ctx.lineTo(centerX + 11, rightArrowY - 8);
      ctx.lineTo(centerX + 19, rightArrowY - 8);
    }
    ctx.closePath();
    ctx.fill();
  }

  /**
   * Публикует Twist сообщение
   * TODO: Нужно реализовать энкодинг CDR для исходящих сообщений
   * Сейчас только парсинг входящих работает через parseCDRBytes
   */
  private publishTwist(linear: number, angular: number): void {
    if (!this.publishCallback || !this.currentRobotName) return;

    try {
      const twist: Twist = {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      };

      // Временно логируем, пока не реализован энкодинг
      logger.debug(
        LOG_CONFIG.PREFIXES.GAMEPAD,
        `Twist: linear=${linear.toFixed(2)}, angular=${angular.toFixed(2)}`
      );

      // TODO: Реализовать encoding в CDR формат
      // const cdrBytes = encodeCDRBytes(twist, twistSchema);
      // this.publishCallback(ROS_TOPICS.CMD_VEL, cdrBytes);
    } catch (err) {
      logger.error(LOG_CONFIG.PREFIXES.GAMEPAD, 'Ошибка публикации Twist:', err);
    }
  }

  /**
   * Проверяет, подключен ли геймпад
   */
  isConnected(): boolean {
    return this.connected;
  }
}

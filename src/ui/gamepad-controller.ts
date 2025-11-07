/**
 * Управление геймпадом для управления роботом
 */

import { GAMEPAD_CONFIG, LOG_CONFIG, ROS_TOPICS } from '../config';
import { logger, serializeTwist } from '../utils';

export class GamepadController {
  private connected = false;
  private intervalId: number | null = null;
  private gamepadIndex: number | null = null;
  private currentRobotName = '';
  private publishCallback: ((topic: string, data: Uint8Array) => Promise<void>) | null = null;
  private armed: boolean = GAMEPAD_CONFIG.ARM.START_ARMED;
  private lastLinear = 0;
  private lastAngular = 0;
  private readonly commandEpsilon = GAMEPAD_CONFIG.COMMAND_EPSILON;
  // Предыдущее состояние оси ARM (для режима LATCH)
  private prevArmAxisActive = false;
  // Сохранение прошлого состояния ARM для отправки стоп-команды при дизарме
  private prevArmed: boolean = GAMEPAD_CONFIG.ARM.START_ARMED;

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
      logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `Кнопка нажата. Connected: ${this.connected}`);
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
   * Обновляет визуальное состояние кнопки и оверлея
   */
  private updateUIState(): void {
    if (!this.connected) {
      this.button.classList.remove('connected');
      this.button.textContent = '🎮 Connect Gamepad';
      this.button.style.backgroundColor = '#2196F3';
      return;
    }

    this.button.classList.add('connected');
    this.button.textContent = this.armed
      ? '⏹ Отключить геймпад (ARM ON)'
      : '⏹ Отключить геймпад (ARM OFF)';
    this.button.style.backgroundColor = this.armed ? '#f44336' : '#FF9800';

    if (this.overlay) {
      this.overlay.style.opacity = this.armed ? '1' : '0.7';
      this.overlay.style.outline = this.armed
        ? '2px solid #4CAF50'
        : '2px dashed rgba(255, 255, 255, 0.5)';
    }
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
        // АВТОМАТИЧЕСКИ ПОДКЛЮЧАЕМ ГЕЙМПАД!
        if (!this.connected) {
          logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, 'Автоподключение обнаруженного геймпада...');
          this.connect();
        }
        break;
      }
    }
  }

  /**
   * Подключает геймпад
   */
  private connect(): void {
    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, '🔍 Начинаем подключение...');
    const gamepads = navigator.getGamepads();
    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `📊 Найдено геймпадов: ${gamepads.length}`);
    let gamepad: Gamepad | null = null;

    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        gamepad = gamepads[i];
        this.gamepadIndex = i;
        logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `✅ Геймпад ${i}: ${gamepad!.id}`);
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

    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, '⏰ Запускаем опрос геймпада...');
    // Запускаем опрос геймпада
    if (this.intervalId) {
      clearInterval(this.intervalId);
    }

    this.intervalId = window.setInterval(() => this.poll(), GAMEPAD_CONFIG.POLL_INTERVAL);
    this.connected = true;
  this.armed = GAMEPAD_CONFIG.ARM.START_ARMED;
    this.lastLinear = 0;
    this.lastAngular = 0;

    // Инициализируем состояние ARM оси, чтобы избежать ложного переключения сразу после подключения
    try {
      const armAxisIndex = GAMEPAD_CONFIG.ARM.AXIS_INDEX;
      const initialArmAxisValue = gamepad.axes[armAxisIndex] ?? 0;
      const armThreshold = GAMEPAD_CONFIG.ARM.THRESHOLD;
      this.prevArmAxisActive = initialArmAxisValue <= -armThreshold;
      this.prevArmed = this.armed;
      logger.debug(
        LOG_CONFIG.PREFIXES.GAMEPAD,
        `ARM init: axis[${armAxisIndex}]=${initialArmAxisValue.toFixed(2)} active=${this.prevArmAxisActive}`
      );
    } catch { /* ignore */ }

    // Обновляем UI
    this.overlay.style.display = 'block';
    this.robotVisual.style.display = 'block';
    this.updateUIState();

    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, `✅ Геймпад активирован: ${gamepad.id}`);
    logger.info(
      LOG_CONFIG.PREFIXES.GAMEPAD,
      `📊 Осей: ${gamepad.axes.length}, Кнопок: ${gamepad.buttons.length}`
    );
    if (!this.armed) {
      logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, '⚠️ Команды не отправляются — ARM выключен');
    } else {
      logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, '🚀 ARM включён автоматически');
    }
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
    this.publishTwist(0, 0, true); // force-stop независимо от ARM

    // Обновляем UI
    this.connected = false;
  this.armed = GAMEPAD_CONFIG.ARM.START_ARMED;
    this.gamepadIndex = null;
    this.lastLinear = 0;
    this.lastAngular = 0;
    this.overlay.style.display = 'none';
    this.robotVisual.style.display = 'none';
    this.updateUIState();

    logger.info(LOG_CONFIG.PREFIXES.GAMEPAD, 'Геймпад отключен');
  }

  /**
   * Опрашивает геймпад и отправляет команды
   */
  private poll(): void {
    const gamepads = navigator.getGamepads();
    const idx = this.gamepadIndex ?? 0;
    const gp = gamepads[idx];
    if (gamepads.length === 0 || !gp) {
      logger.warn(LOG_CONFIG.PREFIXES.GAMEPAD, 'Геймпад потерян, отключение...');
      this.disconnect();
      return;
    }
    const gamepad = gp;

    // === ARM управление через ось ===
    const armAxisIndex = GAMEPAD_CONFIG.ARM.AXIS_INDEX;
    const armThreshold = GAMEPAD_CONFIG.ARM.THRESHOLD;
    if (armAxisIndex >= gamepad.axes.length) {
      logger.warn(
        LOG_CONFIG.PREFIXES.GAMEPAD,
        `ARM ось ${armAxisIndex} отсутствует (доступно осей: ${gamepad.axes.length}). Проверьте конфиг.`
      );
      // Без валидной оси не управляем ARM, просто продолжаем визуализацию
      return;
    }
    const armAxisValue = gamepad.axes[armAxisIndex] ?? 0;
    // ARM активен когда ось < -threshold (т.е. -1.0 при threshold=0.5)
    const armAxisActive = armAxisValue <= -armThreshold;

    if (GAMEPAD_CONFIG.ARM.LATCH) {
      // Режим фиксации: переключение при ЛЮБОМ пересечении порога (туда и обратно)
      if (armAxisActive !== this.prevArmAxisActive) {
        // Состояние оси изменилось — синхронизируем armed с состоянием оси
        this.armed = armAxisActive;
        this.updateUIState();
        logger.info(
          LOG_CONFIG.PREFIXES.GAMEPAD,
          this.armed
            ? `🔥 ARM включён (ось ${armAxisIndex} = ${armAxisValue.toFixed(2)})`
            : `🧊 ARM выключен (ось ${armAxisIndex} = ${armAxisValue.toFixed(2)})`
        );
        if (!this.armed) {
          // При выключении отправляем стоп (force)
          this.publishTwist(0, 0, true);
        }
      }
      this.prevArmAxisActive = armAxisActive;
    } else {
      // Режим удержания: активен пока ось за порогом
      this.armed = armAxisActive;
      if (this.armed !== this.prevArmed) {
        this.updateUIState();
        logger.info(
          LOG_CONFIG.PREFIXES.GAMEPAD,
          this.armed
            ? `🚀 ARM активирован удержанием (ось ${armAxisIndex} = ${armAxisValue.toFixed(2)})`
            : `🛑 ARM деактивирован (ось ${armAxisIndex} = ${armAxisValue.toFixed(2)})`
        );
        if (!this.armed) {
          this.publishTwist(0, 0, true);
        }
      }
    }
    this.prevArmed = this.armed;

    // Получаем значения осей из конфигурации
    const pitch = gamepad.axes[GAMEPAD_CONFIG.PITCH_AXIS]; // По умолчанию: ось 1
    const yaw = gamepad.axes[GAMEPAD_CONFIG.YAW_AXIS];     // По умолчанию: ось 3

    // Применяем мертвую зону
    const pitchValue = Math.abs(pitch) > GAMEPAD_CONFIG.DEADZONE ? pitch : 0;
    const yawValue = Math.abs(yaw) > GAMEPAD_CONFIG.DEADZONE ? yaw : 0;

    // Обновляем визуализацию
    this.updateVisualization(pitchValue, yawValue);

    // Если ARM выключен — не отправляем команды (кроме force stop при переходе)
    if (!this.armed) {
      return; // безопасно игнорируем ввод
    }

    // Если оба значения в мертвой зоне - отправляем единичный стоп (с учетом epsilon)
    if (pitchValue === 0 && yawValue === 0) {
      this.publishTwist(0, 0); // не force — будет подавлено если уже нули
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
   * Публикует Twist сообщение через Zenoh
   */
  private publishTwist(linear: number, angular: number, force = false): void {
    if (!this.publishCallback || !this.currentRobotName) return;
    // Блокировка если не ARM (кроме force-stop)
    if (!this.armed && !force) return;

    // Подавление повторов (epsilon)
    if (!force) {
      const dLinear = Math.abs(linear - this.lastLinear);
      const dAngular = Math.abs(angular - this.lastAngular);
      if (dLinear < this.commandEpsilon && dAngular < this.commandEpsilon) {
        return; // изменения слишком малы
      }
    }

    try {
      const cdrBytes = serializeTwist(linear, angular);
      this.publishCallback(ROS_TOPICS.CMD_VEL, cdrBytes);
      this.lastLinear = linear;
      this.lastAngular = angular;
  // помечать отправку больше не требуется
      logger.debug(
        LOG_CONFIG.PREFIXES.GAMEPAD,
        `Twist${force ? ' (force)' : ''}: linear=${linear.toFixed(2)}, angular=${angular.toFixed(2)}`
      );
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

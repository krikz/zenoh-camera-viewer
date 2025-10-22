# 📋 Код-ревью проекта Zenoh Robot Explorer

> **Дата проверки:** 22 октября 2025
> **Ревьюер:** AI Code Review
> **Версия:** 1.0.0

## 📊 Общая оценка

**Оценка:** ⭐⭐⭐⭐☆ (4/5)

Проект имеет хорошую архитектуру и функциональность, но есть области для улучшения в плане организации кода, типизации и обработки ошибок.

---

## ✅ Сильные стороны

### 1. Архитектура и организация
- ✨ Хорошее разделение ответственности между модулями (`main.ts`, `map.ts`, `idl.ts`)
- ✨ Использование современных технологий (TypeScript, Vite, CDR)
- ✨ Корректная работа с EventSource для потоковых данных

### 2. Функциональность
- ✨ Полноценная интеграция с Zenoh REST API
- ✨ Поддержка множественных типов сообщений ROS
- ✨ Реализация сложных координатных преобразований (map ↔ odom)
- ✨ Визуализация данных на Canvas

### 3. UX
- ✨ Интуитивный интерфейс
- ✨ Поддержка геймпада
- ✨ Сохранение состояния в localStorage

---

## ⚠️ Критические проблемы (высокий приоритет)

### 1. **Отсутствие обработки утечек памяти**

**Проблема:** EventSource соединения могут не закрываться при ошибках или переключении роботов.

**Локация:** `main.ts`, функции `startXxxFeed()`

**Решение:**
```typescript
// Добавить таймауты переподключения
let reconnectTimer: NodeJS.Timeout | null = null;

function startCameraFeed(robotName: string) {
  // Очистка предыдущих таймеров
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }

  // ... существующий код ...

  cameraEventSource.addEventListener("error", (err) => {
    console.error("[SSE Camera] Ошибка:", err);
    
    // Попытка переподключения через 5 секунд
    reconnectTimer = setTimeout(() => {
      if (robotSelect.value === robotName) {
        startCameraFeed(robotName);
      }
    }, 5000);
  });
}
```

### 2. **Отсутствие валидации данных**

**Проблема:** Нет проверки корректности распарсенных данных перед использованием.

**Локация:** Все функции `handleXxxEvent()`

**Решение:**
```typescript
function handleMapEvent(event: MessageEvent) {
  try {
    const sample = JSON.parse(event.data) as { value: string };
    if (!sample.value) return;

    const bytes = base64ToBytes(sample.value);
    const parsed = parseCDRBytes(bytes, occupancyGridSchema, {
      maxSequenceSize: 1_000_000,
    });

    const mapMsg = parsed.payload;
    
    // ✅ Добавить валидацию
    if (!isValidOccupancyGrid(mapMsg)) {
      console.error('[Map] Некорректные данные карты', mapMsg);
      return;
    }

    currentMap = mapMsg;
    renderMap(mapMsg);
  } catch (err) {
    console.error("[SSE Map] Обработка падения:", err);
  }
}

// Вспомогательная функция валидации
function isValidOccupancyGrid(msg: any): msg is OccupancyGrid {
  return (
    msg &&
    typeof msg.info?.width === 'number' &&
    typeof msg.info?.height === 'number' &&
    Array.isArray(msg.data) &&
    msg.data.length === msg.info.width * msg.info.height
  );
}
```

### 3. **Дублирование кода**

**Проблема:** Повторяющаяся логика декодирования base64 → CDR в каждом обработчике.

**Локация:** Все функции `handleXxxEvent()`

**Решение:**
```typescript
// Создать утилитарную функцию
function parseZenohMessage<T>(
  eventData: string,
  schema: any,
  maxSequenceSize: number = 10_000
): T | null {
  try {
    const sample = JSON.parse(eventData) as { value: string };
    if (!sample.value) return null;

    const bytes = base64ToBytes(sample.value);
    const parsed = parseCDRBytes(bytes, schema, { maxSequenceSize });
    return parsed.payload as T;
  } catch (err) {
    console.error('[Parse] Ошибка парсинга:', err);
    return null;
  }
}

// Вспомогательная функция для base64
function base64ToBytes(base64: string): Uint8Array {
  const binaryString = atob(base64);
  const bytes = new Uint8Array(binaryString.length);
  for (let i = 0; i < binaryString.length; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return bytes;
}

// Использование:
function handleMapEvent(event: MessageEvent) {
  const mapMsg = parseZenohMessage<OccupancyGrid>(
    event.data,
    occupancyGridSchema,
    1_000_000
  );
  
  if (!mapMsg || !isValidOccupancyGrid(mapMsg)) return;
  
  currentMap = mapMsg;
  renderMap(mapMsg);
}
```

---

## 🔧 Рекомендации для улучшения (средний приоритет)

### 4. **Улучшение типизации**

**Проблема:** Использование `any` типов, недостаточная типизация схем.

**Локация:** Повсеместно в `main.ts`

**Решение:**
```typescript
// Создать отдельный файл types.ts
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

export interface Header {
  stamp: Time;
  frame_id: string;
}

export interface Time {
  sec: number;
  nanosec: number;
}

export interface Image {
  header: Header;
  height: number;
  width: number;
  encoding: string;
  is_bigendian: number;
  step: number;
  data: number[];
}

// И т.д. для остальных типов
```

### 5. **Вынести константы в конфигурационный файл**

**Проблема:** Хардкод URL и других констант в коде.

**Локация:** `main.ts`

**Решение:**
```typescript
// Создать config.ts
export const CONFIG = {
  ZENOH_REST_BASE: import.meta.env.VITE_ZENOH_URL || 'https://zenoh.robbox.online',
  RECONNECT_DELAY: 5000,
  MAX_RECONNECT_ATTEMPTS: 5,
  MAX_SEQUENCE_SIZES: {
    IMAGE: 300_000,
    MAP: 1_000_000,
    LIDAR: 10_000,
    DEFAULT: 10_000,
  },
  CANVAS: {
    MAP_SIZE: 600,
    LIDAR_SIZE: 600,
  },
} as const;

// Создать .env файл
VITE_ZENOH_URL=https://zenoh.robbox.online
```

### 6. **Разделить main.ts на модули**

**Проблема:** Файл `main.ts` содержит 2000+ строк кода.

**Рекомендуемая структура:**
```
src/
├── main.ts              # Точка входа, инициализация
├── config.ts            # Конфигурация
├── types/
│   ├── ros-messages.ts  # Типы ROS сообщений
│   └── schemas.ts       # CDR схемы
├── services/
│   ├── zenoh-client.ts  # Работа с Zenoh API
│   └── sse-manager.ts   # Управление EventSource
├── renderers/
│   ├── camera.ts        # Отрисовка камеры
│   ├── map.ts           # Отрисовка карты
│   └── lidar.ts         # Отрисовка лидара
├── utils/
│   ├── cdr-parser.ts    # Утилиты парсинга
│   └── transforms.ts    # Координатные преобразования
└── ui/
    ├── controls.ts      # UI контролы
    └── gamepad.ts       # Управление геймпадом
```

### 7. **Добавить логирование**

**Проблема:** Смешанное использование `console.log`, `console.error`, отсутствие уровней логирования.

**Решение:**
```typescript
// logger.ts
enum LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
}

class Logger {
  private level: LogLevel = LogLevel.INFO;

  setLevel(level: LogLevel) {
    this.level = level;
  }

  debug(message: string, ...args: any[]) {
    if (this.level <= LogLevel.DEBUG) {
      console.debug(`[DEBUG] ${message}`, ...args);
    }
  }

  info(message: string, ...args: any[]) {
    if (this.level <= LogLevel.INFO) {
      console.info(`[INFO] ${message}`, ...args);
    }
  }

  warn(message: string, ...args: any[]) {
    if (this.level <= LogLevel.WARN) {
      console.warn(`[WARN] ${message}`, ...args);
    }
  }

  error(message: string, ...args: any[]) {
    if (this.level <= LogLevel.ERROR) {
      console.error(`[ERROR] ${message}`, ...args);
    }
  }
}

export const logger = new Logger();
```

### 8. **Добавить обработку состояния подключения**

**Проблема:** Нет визуального отображения состояния соединения.

**Решение:**
```typescript
enum ConnectionState {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  ERROR = 'error',
}

class ConnectionManager {
  private states = new Map<string, ConnectionState>();
  
  setState(source: string, state: ConnectionState) {
    this.states.set(source, state);
    this.updateUI();
  }
  
  private updateUI() {
    const allConnected = Array.from(this.states.values())
      .every(s => s === ConnectionState.CONNECTED);
    
    statusEl.className = allConnected ? 'status-connected' : 'status-error';
    // ...
  }
}
```

---

## 💡 Предложения для улучшения (низкий приоритет)

### 9. **Добавить тесты**

```typescript
// tests/cdr-parser.test.ts
import { describe, it, expect } from 'vitest';
import { parseZenohMessage } from '../src/utils/cdr-parser';

describe('CDR Parser', () => {
  it('should parse valid base64 message', () => {
    const data = '...'; // mock data
    const result = parseZenohMessage(data, occupancyGridSchema);
    expect(result).toBeDefined();
  });

  it('should return null for invalid data', () => {
    const result = parseZenohMessage('invalid', occupancyGridSchema);
    expect(result).toBeNull();
  });
});
```

### 10. **Добавить CI/CD**

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: '18'
      - run: npm install
      - run: npm run build
      - run: npm test
```

### 11. **Добавить документацию API**

```typescript
/**
 * Подключается к потоку карты робота через Zenoh SSE
 * @param robotName - Имя робота для подключения
 * @throws {Error} Если EventSource не поддерживается браузером
 */
function startMapFeed(robotName: string): void {
  // ...
}
```

### 12. **Оптимизация производительности**

- Использовать `requestAnimationFrame` для отрисовки вместо прямого рендеринга в обработчиках
- Добавить throttling для частых обновлений
- Использовать OffscreenCanvas для фонового рендеринга

```typescript
let animationFrameId: number | null = null;
let pendingRenderData: any = null;

function scheduleRender(data: any) {
  pendingRenderData = data;
  
  if (animationFrameId === null) {
    animationFrameId = requestAnimationFrame(() => {
      if (pendingRenderData) {
        renderMap(pendingRenderData);
        pendingRenderData = null;
      }
      animationFrameId = null;
    });
  }
}
```

---

## 📦 Рекомендации по зависимостям

### Обновить package.json

```json
{
  "name": "zenoh-camera-viewer",
  "version": "1.0.0",
  "description": "Web interface for remote robot control via Zenoh protocol",
  "author": "krikz",
  "license": "ISC",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build",
    "preview": "vite preview",
    "lint": "eslint src --ext ts,tsx",
    "format": "prettier --write \"src/**/*.{ts,tsx,css,html}\"",
    "test": "vitest"
  },
  "devDependencies": {
    "@types/node": "^20.0.0",
    "@typescript-eslint/eslint-plugin": "^6.0.0",
    "@typescript-eslint/parser": "^6.0.0",
    "eslint": "^8.0.0",
    "prettier": "^3.0.0",
    "typescript": "^5.0.0",
    "vitest": "^1.0.0"
  }
}
```

### Добавить ESLint и Prettier

```javascript
// .eslintrc.cjs
module.exports = {
  parser: '@typescript-eslint/parser',
  extends: [
    'eslint:recommended',
    'plugin:@typescript-eslint/recommended',
  ],
  rules: {
    '@typescript-eslint/no-explicit-any': 'warn',
    '@typescript-eslint/no-unused-vars': 'error',
  },
};
```

```json
// .prettierrc
{
  "semi": true,
  "trailingComma": "es5",
  "singleQuote": true,
  "printWidth": 100,
  "tabWidth": 2
}
```

---

## 🎯 План действий

### Краткосрочные задачи (1-2 недели)
1. ✅ Добавить `.gitignore`
2. ✅ Улучшить README.md
3. ⬜ Исправить критические проблемы (#1-3)
4. ⬜ Вынести константы в конфиг
5. ⬜ Добавить типы для всех сообщений

### Среднесрочные задачи (1 месяц)
6. ⬜ Разделить main.ts на модули
7. ⬜ Добавить систему логирования
8. ⬜ Реализовать ConnectionManager
9. ⬜ Настроить ESLint и Prettier

### Долгосрочные задачи (2-3 месяца)
10. ⬜ Добавить unit-тесты
11. ⬜ Настроить CI/CD
12. ⬜ Оптимизировать производительность
13. ⬜ Добавить документацию JSDoc

---

## 📈 Метрики кода

| Метрика | Значение | Рекомендация |
|---------|----------|--------------|
| Размер main.ts | 2043 строки | < 500 строк |
| Использование `any` | ~30 раз | < 5 раз |
| Покрытие тестами | 0% | > 70% |
| Дублирование кода | ~15% | < 5% |
| Цикломатическая сложность | Средняя | Низкая |

---

## 🎓 Заключение

Проект демонстрирует хорошее понимание работы с Zenoh и ROS, имеет рабочую реализацию сложной функциональности. Основные улучшения требуются в области:

1. **Организации кода** - разделение на модули
2. **Надежности** - обработка ошибок и валидация
3. **Поддерживаемости** - типизация и документация
4. **Тестирования** - добавление автотестов

После выполнения рекомендаций проект будет готов для production использования и дальнейшего масштабирования.

**Рекомендуемое время на исправления:** 2-3 недели активной разработки

---

*Документ создан: 22 октября 2025*
*Версия: 1.0*

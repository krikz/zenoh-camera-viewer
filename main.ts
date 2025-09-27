import { parseCDRBytes } from "@mono424/cdr-ts";
import { OccupancyGrid, occupancyGridSchema } from "./map";
import { MessageWriter } from "@lichtblick/omgidl-serialization";
import { navigateToPoseDefinition, NavigateToPoseGoal } from "./idl";

// Создаём writer один раз (можно кэшировать)
const goalWriter = new MessageWriter(
  "nav2_msgs::NavigateToPose_Goal",
  navigateToPoseDefinition,
  { kind: 0x01 } // XCDR не требуется
);

// DOM элементы
const robotSelect = document.getElementById("robotSelect") as HTMLSelectElement;
const exploreBtn = document.getElementById("exploreBtn") as HTMLButtonElement;
const statusEl = document.getElementById("status") as HTMLElement;
const mapCanvas = document.getElementById("mapCanvas") as HTMLCanvasElement;
const cameraCanvas = document.getElementById(
  "cameraCanvas"
) as HTMLCanvasElement;

const mapCtx = mapCanvas.getContext("2d", { willReadFrequently: true })!;
const cameraCtx = cameraCanvas.getContext("2d", { willReadFrequently: true })!;

// Базовый URL — исправленные пробелы
const ZENOH_REST_BASE = "https://zenoh.robbox.online";

// Схема для sensor_msgs/Image
const imageSchema = {
  type: "dictionary",
  items: {
    header: {
      index: 0,
      value: {
        type: "dictionary",
        items: {
          stamp: {
            index: 0,
            value: {
              type: "dictionary",
              items: {
                sec: {
                  index: 0,
                  value: { type: "uint", len: 32, format: "number" },
                },
                nanosec: {
                  index: 1,
                  value: { type: "uint", len: 32, format: "number" },
                },
              },
            },
          },
          frame_id: { index: 1, value: { type: "string" } },
        },
      },
    },
    height: { index: 1, value: { type: "uint", len: 32, format: "number" } },
    width: { index: 2, value: { type: "uint", len: 32, format: "number" } },
    encoding: { index: 3, value: { type: "string" } },
    is_bigendian: {
      index: 4,
      value: { type: "int", len: 8, format: "number" },
    },
    step: { index: 5, value: { type: "uint", len: 32, format: "number" } },
    data: {
      index: 6,
      value: {
        type: "sequence",
        itemSchema: { type: "uint", len: 8, format: "number" },
      },
    },
  },
} as const;

// SSE источники
let cameraEventSource: EventSource | null = null;
let mapEventSource: EventSource | null = null;

async function fetchRobots() {
  try {
    const url = `${ZENOH_REST_BASE}/robots/**`;
    const resp = await fetch(url);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);

    const data: Array<{ key: string }> = await resp.json();

    // Извлекаем уникальные имена роботов
    const robotsSet = new Set<string>();
    for (const item of data) {
      const parts = item.key.split("/");
      if (parts.length > 1 && parts[0] === "robots") {
        const robotName = parts[1];
        if (robotName) robotsSet.add(robotName);
      }
    }

    const robots = Array.from(robotsSet).sort();

    robotSelect.innerHTML = "";
    if (robots.length === 0) {
      robotSelect.innerHTML = "<option>Роботы не найдены</option>";
      return;
    }

    robotSelect.innerHTML = '<option value="">-- Выберите робота --</option>';
    robots.forEach((name) => {
      const opt = document.createElement("option");
      opt.value = name;
      opt.textContent = name;
      robotSelect.appendChild(opt);
    });

    robotSelect.onchange = () => {
      const robotName = robotSelect.value;
      if (robotName) {
        statusEl.textContent = `📡 Подключение к ${robotName}...`;
        setupRobotFeeds(robotName);
      } else {
        cleanupRobotFeeds();
        statusEl.textContent = "Выберите робота";
      }
    };

    // Автоматическая загрузка при наличии выбора
    if (robotSelect.value) {
      setupRobotFeeds(robotSelect.value);
    }
  } catch (err) {
    console.error("[REST] Ошибка получения списка роботов:", err);
    robotSelect.innerHTML = `<option>Ошибка: ${
      err instanceof Error ? err.message : "Неизвестная ошибка"
    }</option>`;
  }
}

function setupRobotFeeds(robotName: string) {
  cleanupRobotFeeds();
  startCameraFeed(robotName);
  startMapFeed(robotName);
}

function cleanupRobotFeeds() {
  if (cameraEventSource) {
    cameraEventSource.close();
    cameraEventSource = null;
  }
  if (mapEventSource) {
    mapEventSource.close();
    mapEventSource = null;
  }
}

function startCameraFeed(robotName: string) {
  const key = `robots/${robotName}/robot_cam`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    statusEl.textContent =
      "⚠️ Браузер не поддерживает SSE. Используйте polling.";
    return;
  }

  cameraEventSource = new EventSource(url);

  cameraEventSource.onopen = () => {
    console.log("[SSE] Подключено к камере:", key);
    statusEl.textContent = `🎥 Получение видео...`;
  };

  cameraEventSource.onerror = (err) => {
    console.error("[SSE Camera] Ошибка:", err);
    statusEl.textContent = "⚠️ Ошибка SSE камеры";
    cameraEventSource?.close();
    cameraEventSource = null;
  };
  cameraEventSource.addEventListener("PUT", (event: MessageEvent<any>) => {
    try {
      const sample = JSON.parse(event.data) as {
        value: string;
      };

      if (!sample.value) return;

      // Декодируем base64
      const binaryString = atob(sample.value);
      const bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }

      // Парсим CDR
      const parsed = parseCDRBytes(bytes, imageSchema, {
        maxSequenceSize: 300_000,
      });
      const msg = parsed.payload;

      statusEl.textContent = `🎥 ${msg.width}x${msg.height}, ${msg.encoding}`;
      renderImage(msg);
    } catch (err) {
      console.error("[SSE Camera] Обработка падения:", err);
      statusEl.textContent = "⚠️ Ошибка обработки кадра";
    }
  });
}

function startMapFeed(robotName: string) {
  const key = `robots/${robotName}/map`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    console.warn(
      "EventSource не поддерживается. Карта не будет обновляться автоматически."
    );
    return;
  }

  mapEventSource = new EventSource(url);

  mapEventSource.onopen = () => {
    console.log("[SSE] Подключено к карте:", key);
  };

  mapEventSource.onerror = (err) => {
    console.error("[SSE Map] Ошибка:", err);
    mapEventSource?.close();
    mapEventSource = null;
  };

  mapEventSource.addEventListener("PUT", (event: MessageEvent<any>) => {
    try {
      const sample = JSON.parse(event.data) as { value: string };

      if (!sample.value) return;

      // Декодируем base64
      const binaryString = atob(sample.value);
      const bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }

      // Парсим CDR
      const parsed = parseCDRBytes(bytes, occupancyGridSchema, {
        maxSequenceSize: 1_000_000,
      });

      const mapMsg = parsed.payload;
      renderMap(mapMsg);
    } catch (err) {
      console.error("[SSE Map] Обработка падения:", err);
    }
  });
}

function renderImage(msg: any) {
  const { width, height, encoding, data, step } = msg;

  if (!width || !height || !data || data.length === 0) return;

  // Проверка размера
  if (step * height !== data.length) {
    console.warn(
      `[Render] Размер не совпадает: step*height=${step}*${height}=${
        step * height
      }, data.length=${data.length}`
    );
  }

  cameraCanvas.width = width;
  cameraCanvas.height = height;
  const imageData = cameraCtx.createImageData(width, height);

  // Конвертируем в RGBA
  for (let i = 0; i < data.length; i += 1) {
    const pixelIndex = Math.floor(i / (step / width)) * 4;

    if (encoding === "bgr8") {
      imageData.data[pixelIndex + 0] = data[i + 2]; // R
      imageData.data[pixelIndex + 1] = data[i + 1]; // G
      imageData.data[pixelIndex + 2] = data[i + 0]; // B
      imageData.data[pixelIndex + 3] = 255; // A
      i += 2;
    } else if (encoding === "rgb8") {
      imageData.data[pixelIndex + 0] = data[i + 0]; // R
      imageData.data[pixelIndex + 1] = data[i + 1]; // G
      imageData.data[pixelIndex + 2] = data[i + 2]; // B
      imageData.data[pixelIndex + 3] = 255; // A
      i += 2;
    } else if (encoding === "mono8") {
      const v = data[i];
      imageData.data[pixelIndex + 0] = v;
      imageData.data[pixelIndex + 1] = v;
      imageData.data[pixelIndex + 2] = v;
      imageData.data[pixelIndex + 3] = 255;
    } else {
      console.warn("Неподдерживаемая кодировка:", encoding);
      return;
    }
  }

  cameraCtx.putImageData(imageData, 0, 0);
}

// Запуск
fetchRobots().catch(console.error);

function findFrontiers(mapMsg: OccupancyGrid): Array<{ x: number; y: number }> {
  const { width, height } = mapMsg.info;
  const data = mapMsg.data;

  if (!data || width === 0 || height === 0) {
    console.warn("Некорректные данные карты:", {
      width,
      height,
      hasData: !!data,
    });
    return [];
  }

  const frontiers: Array<{ x: number; y: number }> = [];

  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      const idx = y * width + x;

      // Проверяем, что индекс в пределах массива
      if (idx >= data.length) continue;

      // Только свободные ячейки
      if (data[idx] !== 0) continue;

      // Проверяем соседей
      const neighbors = [
        data[idx - width - 1],
        data[idx - width],
        data[idx - width + 1],
        data[idx - 1],
        /* self */ data[idx + 1],
        data[idx + width - 1],
        data[idx + width],
        data[idx + width + 1],
      ];

      if (neighbors.some((v) => v === 255)) {
        frontiers.push({ x, y });
      }
    }
  }

  return frontiers;
}

function chooseClosestFrontier(
  frontiers: Array<{ x: number; y: number }>,
  robotCellX: number,
  robotCellY: number
): { x: number; y: number } | null {
  let closest = null;
  let minDist = Infinity;

  for (const f of frontiers) {
    const dx = f.x - robotCellX;
    const dy = f.y - robotCellY;
    const dist = dx * dx + dy * dy;

    if (dist < minDist) {
      minDist = dist;
      closest = f;
    }
  }

  return closest;
}

async function sendGoal(robotName: string, goalX: number, goalY: number) {
  try {
    // Формируем цель
    const goal: NavigateToPoseGoal = {
      header: {
        stamp: {
          sec: Math.floor(Date.now() / 1000),
          nanosec: (Date.now() % 1000) * 1_000_000,
        },
        frame_id: "map",
      },
      pose: {
        position: { x: goalX, y: goalY, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
      behavior_tree: "",
    };

    // Сериализуем в CDR
    const cdrBytes: Uint8Array = goalWriter.writeMessage(goal);

    // URL для action (без пробелов!)
    const key = `robots/${robotName}/navigate_to_pose/_action/send_goal`;
    const url = `${ZENOH_REST_BASE}/${key}`;

    const response = await fetch(url, {
      method: "POST",
      headers: {
        "Content-Type": "application/octet-stream",
      },
      body: cdrBytes,
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${await response.text()}`);
    }

    console.log(`✅ Цель отправлена: (${goalX}, ${goalY})`);
    statusEl.textContent = `🧭 Цель отправлена: (${goalX.toFixed(
      2
    )}, ${goalY.toFixed(2)})`;
  } catch (err) {
    console.error("❌ Ошибка отправки цели:", err);
    statusEl.textContent = `⚠️ Ошибка отправки цели: ${
      err instanceof Error ? err.message : String(err)
    }`;
  }
}

async function startExploration(robotName: string) {
  const visitedFrontiers = new Set<string>();

  while (explorationActive) {
    try {
      // Карта теперь приходит через SSE, но на всякий случай оставим fetchMap
      // как fallback если SSE не работает
      const map = await fetchMap(robotName);
      if (!map) continue;

      const frontiers = findFrontiers(map);
      const unvisited = frontiers.filter(
        (f) => !visitedFrontiers.has(`${f.x},${f.y}`)
      );

      if (unvisited.length === 0) {
        statusEl.textContent = "✅ Исследование завершено";
        explorationActive = false;
        exploreBtn.textContent = "▶️ Explore";
        break;
      }

      // TODO: Получить реальную позицию робота
      const goalCell = chooseClosestFrontier(unvisited, 100, 100);
      if (!goalCell) break;

      visitedFrontiers.add(`${goalCell.x},${goalCell.y}`);

      const goalX =
        goalCell.x * map.info.resolution + map.info.origin.position.x;
      const goalY =
        goalCell.y * map.info.resolution + map.info.origin.position.y;

      await sendGoal(robotName, goalX, goalY);

      // Ждём 5 секунд перед следующим шагом
      await new Promise((r) => setTimeout(r, 5000));
    } catch (err) {
      console.error("Ошибка исследования:", err);
      statusEl.textContent = "⚠️ Ошибка, продолжается...";
      await new Promise((r) => setTimeout(r, 5000));
    }
  }
}

let explorationActive = false;

exploreBtn.onclick = () => {
  if (explorationActive) {
    explorationActive = false;
    exploreBtn.textContent = "▶️ Explore";
    exploreBtn.classList.remove("stop");
    exploreBtn.classList.add("play");
    statusEl.textContent = "Исследование остановлено";
  } else {
    const robotName = robotSelect.value;
    if (!robotName) {
      statusEl.textContent = "❌ Сначала выберите робота";
      return;
    }
    explorationActive = true;
    exploreBtn.textContent = "⏹ Stop";
    exploreBtn.classList.remove("play");
    exploreBtn.classList.add("stop");
    statusEl.textContent = "Исследование запущено...";
    startExploration(robotName);
  }
};

function resizeCanvases() {
  // Карта — занимает всё свободное место
  const rect = mapCanvas.parentElement!.getBoundingClientRect();
  mapCanvas.width = rect.width;
  mapCanvas.height = rect.height;

  // Камера — фиксированная ширина, высота по пропорции
  const camWidth = 240;
  cameraCanvas.width = camWidth;
  cameraCanvas.height = camWidth * 0.75; // 4:3
}

window.addEventListener("load", resizeCanvases);
window.addEventListener("resize", resizeCanvases);

async function fetchMap(robotName: string): Promise<OccupancyGrid | null> {
  try {
    const key = `robots/${robotName}/map`;
    const url = `${ZENOH_REST_BASE}/${key}`;
    const resp = await fetch(url);
    const samples: Array<{ value: string }> = await resp.json();

    if (!samples?.length || !samples[0].value) return null;

    const base64 = samples[0].value;
    const bytes = Uint8Array.from(atob(base64), (c) => c.charCodeAt(0));

    const parsed = parseCDRBytes(bytes, occupancyGridSchema, {
      maxSequenceSize: 1_000_000,
    });

    return parsed.payload;
  } catch (err) {
    console.error("[Map] Ошибка:", err);
    return null;
  }
}

function renderMap(msg: OccupancyGrid) {
  const { width, height } = msg.info;
  const data = msg.data;

  // Масштабируем карту под размер canvas
  const scale = Math.min(mapCanvas.width / width, mapCanvas.height / height);

  const drawWidth = width * scale;
  const drawHeight = height * scale;
  const offsetX = (mapCanvas.width - drawWidth) / 2;
  const offsetY = (mapCanvas.height - drawHeight) / 2;

  mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);

  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = y * width + x;
      let r = 0,
        g = 0,
        b = 0;

      if (data[i] === -1) r = g = b = 127; // Unknown
      else if (data[i] === 0) r = g = b = 255; // Free
      else r = g = b = Math.max(0, 255 - data[i]); // Occupied

      mapCtx.fillStyle = `rgb(${r},${g},${b})`;
      mapCtx.fillRect(offsetX + x * scale, offsetY + y * scale, scale, scale);
    }
  }
}

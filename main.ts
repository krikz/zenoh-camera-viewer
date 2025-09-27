import { parseCDRBytes } from "@mono424/cdr-ts";

// DOM элементы
const robotSelect = document.getElementById("robotSelect") as HTMLSelectElement;
const statusEl = document.getElementById("status") as HTMLElement;
const canvas = document.getElementById("cameraCanvas") as HTMLCanvasElement;
const ctx = canvas.getContext("2d", { willReadFrequently: true })!;

// Базовый URL — ТОЧНО как ты сказал
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

// Убираем дублирование при обновлении
let pollInterval: any = null;

async function fetchRobots() {
  try {
    // 🔥 ТОЧНО ТАК, КАК ТЫ СКАЗАЛ
    const url = `${ZENOH_REST_BASE}/robots/**`;
    const resp = await fetch(url);
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);

    const data: Array<{ key: string }> = await resp.json();

    // Извлекаем уникальные имена роботов (второй уровень)
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
        statusEl.textContent = `Загрузка камеры: ${robotName}`;
        startPollingCamera(robotName);
      } else {
        statusEl.textContent = "Выберите робота";
        clearInterval(pollInterval);
      }
    };

    // Если был выбран робот — запустить сразу
    if (robotSelect.value) {
      startPollingCamera(robotSelect.value);
    }
  } catch (err) {
    console.error("[REST] Ошибка получения списка роботов:", err);
    robotSelect.innerHTML = `<option>Ошибка: ${
      err instanceof Error ? err.message : "Неизвестная ошибка"
    }</option>`;
  }
}

function startPollingCamera(robotName: string) {
  clearInterval(pollInterval);

  async function poll() {
    try {
      const key = `robots/${robotName}/robot_cam`;
      const url = `${ZENOH_REST_BASE}/${key}`;

      const resp = await fetch(url);
      if (!resp.ok) {
        statusEl.textContent = "❌ Нет данных с камеры";
        return;
      }

      // Правильная типизация ответа
      const samples: Array<{
        key: string;
        value: string; // Это строка Base64!
        encoding: string;
        timestamp: string;
      }> = await resp.json();

      if (!samples || samples.length === 0 || !samples[0].value) {
        statusEl.textContent = "⚠️ Пустое изображение";
        return;
      }

      const base64Data = samples[0].value; // ⬅️ Вот он: base64 в поле `value`
      const binaryString = atob(base64Data);
      const bytes = new Uint8Array(binaryString.length);
      for (let i = 0; i < binaryString.length; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }

      // Парсим CDR
      const parsed = parseCDRBytes(bytes, imageSchema,{maxSequenceSize:300000});
      const msg = parsed.payload;

      statusEl.textContent = `✅ ${msg.width}x${msg.height}, ${msg.encoding}`;

      renderImage(msg);
    } catch (err) {
      console.error("[Camera Poll] Ошибка:", err);
      statusEl.textContent = "⚠️ Ошибка загрузки";
    }
  }

  // Первый раз сразу
  poll();

  // Затем каждые 500 мс
  pollInterval = setInterval(poll, 500);
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

  canvas.width = width;
  canvas.height = height;
  const imageData = ctx.createImageData(width, height);

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

  ctx.putImageData(imageData, 0, 0);
}

// Запуск
fetchRobots().catch(console.error);

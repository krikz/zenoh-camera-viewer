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
const lidarCanvas = document.getElementById("lidarCanvas") as HTMLCanvasElement;
const cameraCanvas = document.getElementById(
  "cameraCanvas"
) as HTMLCanvasElement;

const mapCtx = mapCanvas.getContext("2d", { willReadFrequently: true })!;
const lidarCtx = lidarCanvas.getContext("2d", { willReadFrequently: true })!;
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

// Схема для sensor_msgs/LaserScan
const laserScanSchema = {
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
    angle_min: {
      index: 1,
      value: { type: "float", len: 32, format: "number" },
    },
    angle_max: {
      index: 2,
      value: { type: "float", len: 32, format: "number" },
    },
    angle_increment: {
      index: 3,
      value: { type: "float", len: 32, format: "number" },
    },
    time_increment: {
      index: 4,
      value: { type: "float", len: 32, format: "number" },
    },
    scan_time: {
      index: 5,
      value: { type: "float", len: 32, format: "number" },
    },
    range_min: {
      index: 6,
      value: { type: "float", len: 32, format: "number" },
    },
    range_max: {
      index: 7,
      value: { type: "float", len: 32, format: "number" },
    },
    ranges: {
      index: 8,
      value: {
        type: "sequence",
        itemSchema: { type: "float", len: 32, format: "number" },
      },
    },
    intensities: {
      index: 9,
      value: {
        type: "sequence",
        itemSchema: { type: "float", len: 32, format: "number" },
      },
    },
  },
} as const;

// Схема для nav_msgs/Odometry
const odometrySchema = {
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
    child_frame_id: { index: 1, value: { type: "string" } },
    pose: {
      index: 2,
      value: {
        type: "dictionary",
        items: {
          pose: {
            index: 0,
            value: {
              type: "dictionary",
              items: {
                position: {
                  index: 0,
                  value: {
                    type: "dictionary",
                    items: {
                      x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                      y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                      z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                    },
                  },
                },
                orientation: {
                  index: 1,
                  value: {
                    type: "dictionary",
                    items: {
                      x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                      y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                      z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                      w: { index: 3, value: { type: "float", len: 64, format: "number" } },
                    },
                  },
                },
              },
            },
          },
        },
      },
    },
  },
} as const;

// Схема для tf2_msgs/TFMessage
const tfSchema = {
  type: "dictionary",
  items: {
    transforms: {
      index: 0,
      value: {
        type: "sequence",
        itemSchema: {
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
            child_frame_id: { index: 1, value: { type: "string" } },
            transform: {
              index: 2,
              value: {
                type: "dictionary",
                items: {
                  translation: {
                    index: 0,
                    value: {
                      type: "dictionary",
                      items: {
                        x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                        y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                        z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                      },
                    },
                  },
                  rotation: {
                    index: 1,
                    value: {
                      type: "dictionary",
                      items: {
                        x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                        y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                        z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                        w: { index: 3, value: { type: "float", len: 64, format: "number" } },
                      },
                    },
                  },
                },
              },
            },
          },
        },
      },
    },
  },
} as const;

// Схема для nav_msgs/Path
const pathSchema = {
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
    poses: {
      index: 1,
      value: {
        type: "sequence",
        itemSchema: {
          type: "dictionary",
          items: {
            pose: {
              index: 0,
              value: {
                type: "dictionary",
                items: {
                  position: {
                    index: 0,
                    value: {
                      type: "dictionary",
                      items: {
                        x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                        y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                        z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                      },
                    },
                  },
                  orientation: {
                    index: 1,
                    value: {
                      type: "dictionary",
                      items: {
                        x: { index: 0, value: { type: "float", len: 64, format: "number" } },
                        y: { index: 1, value: { type: "float", len: 64, format: "number" } },
                        z: { index: 2, value: { type: "float", len: 64, format: "number" } },
                        w: { index: 3, value: { type: "float", len: 64, format: "number" } },
                      },
                    },
                  },
                },
              },
            },
          },
        },
      },
    },
  },
} as const;

// SSE источники
let cameraEventSource: EventSource | null = null;
let mapEventSource: EventSource | null = null;
let lidarEventSource: EventSource | null = null;
let odometryEventSource: EventSource | null = null;
let tfEventSource: EventSource | null = null;
let planEventSource: EventSource | null = null;

// Текущая карта для отрисовки лидара
let currentMap: OccupancyGrid | null = null;

// Текущая позиция робота в системе координат карты (map)
let robotPosition = { x: 0, y: 0, theta: 0 };

// Текущая траектория из /plan
let currentPlan: any[] = [];

// Трансформация от map к odom
let mapToOdom = {
  translation: { x: 0, y: 0, z: 0 },
  rotation: { x: 0, y: 0, z: 0, w: 1 }
};

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
  startLidarFeed(robotName);
  startOdometryFeed(robotName);
  startTfFeed(robotName);
  startPlanFeed(robotName);
}

function cleanupRobotFeeds() {
  // Закрываем все EventSource
  if (cameraEventSource) {
    cameraEventSource.removeEventListener("PUT", handleCameraEvent);
    cameraEventSource.close();
    cameraEventSource = null;
  }
  if (mapEventSource) {
    mapEventSource.removeEventListener("PUT", handleMapEvent);
    mapEventSource.close();
    mapEventSource = null;
  }
  if (lidarEventSource) {
    lidarEventSource.removeEventListener("PUT", handleLidarEvent);
    lidarEventSource.close();
    lidarEventSource = null;
  }
  if (odometryEventSource) {
    odometryEventSource.removeEventListener("PUT", handleOdometryEvent);
    odometryEventSource.close();
    odometryEventSource = null;
  }
  if (tfEventSource) {
    tfEventSource.removeEventListener("PUT", handleTfEvent);
    tfEventSource.close();
    tfEventSource = null;
  }
  if (planEventSource) {
    planEventSource.removeEventListener("PUT", handlePlanEvent);
    planEventSource.close();
    planEventSource = null;
  }
  
  // Очищаем лидарный холст при смене робота
  if (lidarCtx) {
    lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
  }
  
  // Очищаем текущую траекторию
  currentPlan = [];
}

// Обработчики событий
function handleCameraEvent(event: MessageEvent) {
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
}

function handleMapEvent(event: MessageEvent) {
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
    currentMap = mapMsg;
    
    // Очищаем лидарный холст при обновлении карты
    if (lidarCtx) {
      lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
    }
    
    renderMap(mapMsg);
  } catch (err) {
    console.error("[SSE Map] Обработка падения:", err);
  }
}

function handleLidarEvent(event: MessageEvent) {
  try {
    const sample = JSON.parse(event.data) as { value: string };
    if (!sample.value || !currentMap) return;

    // Декодируем base64
    const binaryString = atob(sample.value);
    const bytes = new Uint8Array(binaryString.length);
    for (let i = 0; i < binaryString.length; i++) {
      bytes[i] = binaryString.charCodeAt(i);
    }

    // Парсим CDR
    const parsed = parseCDRBytes(bytes, laserScanSchema, {
      maxSequenceSize: 10_000,
    });

    const scan = parsed.payload;
    renderLidar(scan);
  } catch (err) {
    console.error("[SSE Lidar] Обработка падения:", err);
  }
}

function handleOdometryEvent(event: MessageEvent) {
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
    const parsed = parseCDRBytes(bytes, odometrySchema, {
      maxSequenceSize: 10_000,
    });

    const odom = parsed.payload;
    
    // Извлекаем позицию в системе odom
    const odomX = odom.pose.pose.position.x;
    const odomY = odom.pose.pose.position.y;
    const q = odom.pose.pose.orientation;
    
    // Преобразуем кватернион в угол для 2D
    const odomTheta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    
    // Преобразуем позицию из системы odom в систему map
    const mapPose = transformOdomToMap(odomX, odomY, odomTheta);
    
    robotPosition = mapPose;
    
    // Перерисовываем лидар
    if (currentMap) {
      renderLidar(null); // null означает, что мы перерисовываем без новых данных лидара
    }
  } catch (err) {
    console.error("[SSE Odometry] Обработка падения:", err);
  }
}

function handleTfEvent(event: MessageEvent) {
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
    const parsed = parseCDRBytes(bytes, tfSchema, {
      maxSequenceSize: 10_000,
    });

    const tfMsg = parsed.payload;
    
    // Ищем трансформацию от map к odom
    for (const transform of tfMsg.transforms) {
      if (transform.header.frame_id === "map" && transform.child_frame_id === "odom") {
        mapToOdom = {
          translation: transform.transform.translation,
          rotation: transform.transform.rotation
        };
        break;
      }
    }
  } catch (err) {
    console.error("[SSE TF] Обработка падения:", err);
  }
}

function handlePlanEvent(event: MessageEvent) {
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
    const parsed = parseCDRBytes(bytes, pathSchema, {
      maxSequenceSize: 10_000,
    });

    const path = parsed.payload;
    
    // Сохраняем траекторию
    currentPlan = path.poses.map((pose: any) => ({
      x: pose.pose.position.x,
      y: pose.pose.position.y
    }));
    
    // Перерисовываем лидар, чтобы обновить траекторию
    if (currentMap) {
      renderLidar(null);
    }
  } catch (err) {
    console.error("[SSE Plan] Обработка падения:", err);
  }
}

// Преобразуем позицию из системы odom в систему map
function transformOdomToMap(odomX: number, odomY: number, odomTheta: number) {
  // Трансформация из map в odom
  const { x: tx, y: ty } = mapToOdom.translation;
  const { x: qx, y: qy, z: qz, w: qw } = mapToOdom.rotation;
  
  // Вычисляем угол поворота трансформации map->odom
  const mapToOdomTheta = Math.atan2(
    2 * (qw * qz + qx * qy), 
    1 - 2 * (qy * qy + qz * qz)
  );
  
  // Обратная трансформация: из odom в map
  // Сначала применяем обратный поворот, затем обратное смещение
  const cosTheta = Math.cos(-mapToOdomTheta);
  const sinTheta = Math.sin(-mapToOdomTheta);
  
  // Смещение в системе map
  const mapX = tx + cosTheta * odomX - sinTheta * odomY;
  const mapY = ty + sinTheta * odomX + cosTheta * odomY;
  const mapTheta = odomTheta - mapToOdomTheta;
  
  return { x: mapX, y: mapY, theta: mapTheta };
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

  cameraEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к камере:", key);
    statusEl.textContent = `🎥 Получение видео...`;
  });

  cameraEventSource.addEventListener("error", (err) => {
    console.error("[SSE Camera] Ошибка:", err);
    statusEl.textContent = "⚠️ Ошибка SSE камеры";
    cameraEventSource?.close();
    cameraEventSource = null;
  });

  cameraEventSource.addEventListener("PUT", handleCameraEvent);
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

  mapEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к карте:", key);
  });

  mapEventSource.addEventListener("error", (err) => {
    console.error("[SSE Map] Ошибка:", err);
    mapEventSource?.close();
    mapEventSource = null;
  });

  mapEventSource.addEventListener("PUT", handleMapEvent);
}

function startLidarFeed(robotName: string) {
  const key = `robots/${robotName}/scan`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    console.warn(
      "EventSource не поддерживается. Лидар не будет отображаться."
    );
    return;
  }

  lidarEventSource = new EventSource(url);

  lidarEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к лидару:", key);
  });

  lidarEventSource.addEventListener("error", (err) => {
    console.error("[SSE Lidar] Ошибка:", err);
    lidarEventSource?.close();
    lidarEventSource = null;
  });

  lidarEventSource.addEventListener("PUT", handleLidarEvent);
}

function startOdometryFeed(robotName: string) {
  const key = `robots/${robotName}/diff_drive_base_controller/odom`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    console.warn(
      "EventSource не поддерживается. Позиция робота не будет отображаться."
    );
    return;
  }

  odometryEventSource = new EventSource(url);

  odometryEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к одометрии:", key);
  });

  odometryEventSource.addEventListener("error", (err) => {
    console.error("[SSE Odometry] Ошибка:", err);
    odometryEventSource?.close();
    odometryEventSource = null;
  });

  odometryEventSource.addEventListener("PUT", handleOdometryEvent);
}

function startTfFeed(robotName: string) {
  const key = `robots/${robotName}/tf`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    console.warn(
      "EventSource не поддерживается. TF не будет обрабатываться."
    );
    return;
  }

  tfEventSource = new EventSource(url);

  tfEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к TF:", key);
  });

  tfEventSource.addEventListener("error", (err) => {
    console.error("[SSE TF] Ошибка:", err);
    tfEventSource?.close();
    tfEventSource = null;
  });

  tfEventSource.addEventListener("PUT", handleTfEvent);
}

function startPlanFeed(robotName: string) {
  const key = `robots/${robotName}/plan`;
  const url = `${ZENOH_REST_BASE}/${key}`;

  // Проверка поддержки EventSource
  if (typeof EventSource === "undefined") {
    console.warn(
      "EventSource не поддерживается. План не будет отображаться."
    );
    return;
  }

  planEventSource = new EventSource(url);

  planEventSource.addEventListener("open", () => {
    console.log("[SSE] Подключено к плану:", key);
  });

  planEventSource.addEventListener("error", (err) => {
    console.error("[SSE Plan] Ошибка:", err);
    planEventSource?.close();
    planEventSource = null;
  });

  planEventSource.addEventListener("PUT", handlePlanEvent);
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

      // Получаем текущую позицию робота в системе карты
      const robotCellX = Math.round((robotPosition.x - map.info.origin.position.x) / map.info.resolution);
      const robotCellY = Math.round((robotPosition.y - map.info.origin.position.y) / map.info.resolution);

      const goalCell = chooseClosestFrontier(unvisited, robotCellX, robotCellY);
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
  lidarCanvas.width = rect.width;
  lidarCanvas.height = rect.height;

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
  const { width, height, resolution, origin } = msg.info;
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

function renderLidar(scan: any) {
  if (!currentMap || !lidarCtx) return;
  
  // Очищаем только лидарный холст
  lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
  
  const { width, height, resolution, origin } = currentMap.info;
  let ranges = [];
  let angle_min, angle_max, angle_increment;
  
  // Если scan null, то мы перерисовываем без новых данных лидара (только позиция робота изменилась)
  if (scan) {
    ({ ranges, angle_min, angle_max, angle_increment } = scan);
  } else {
    // Используем данные из последнего скана или заглушки
    if (lastLidarScan) {
      ({ ranges, angle_min, angle_max, angle_increment } = lastLidarScan);
    } else {
      return; // Нечего рисовать
    }
  }
  
  // Сохраняем для последующих перерисовок при изменении позиции робота
  if (scan) {
    lastLidarScan = scan;
  }

  // Масштаб и смещение из renderMap
  const scale = Math.min(mapCanvas.width / width, mapCanvas.height / height);
  const drawWidth = width * scale;
  const drawHeight = height * scale;
  const offsetX = (mapCanvas.width - drawWidth) / 2;
  const offsetY = (mapCanvas.height - drawHeight) / 2;

  // Настройки отрисовки лидара
  lidarCtx.strokeStyle = "rgba(0, 255, 0, 0.7)";
  lidarCtx.fillStyle = "rgba(0, 255, 0, 0.7)";
  lidarCtx.lineWidth = 1;

  const angleCount = ranges.length;
  
  // Отрисовка векторов направления для лидара (каждый 5-й луч)
  lidarCtx.strokeStyle = "rgba(255, 165, 0, 0.3)";
  lidarCtx.lineWidth = 0.5;
  
  for (let i = 0; i < angleCount; i += 5) {
    const range = ranges[i];
    if (range === undefined || range === null || range === Infinity || 
        range < (scan?.range_min || 0) || range > (scan?.range_max || Infinity)) continue;

    // Поворот на 90 градусов против часовой стрелки
    const angle = angle_min + i * angle_increment + Math.PI/2;
    
    // Преобразуем в координаты относительно робота
    const x = range * Math.cos(angle);
    const y = range * Math.sin(angle);

    // Мировые координаты точки лидара
    const worldX = robotPosition.x + x;
    const worldY = robotPosition.y + y;

    // Правильное преобразование в координаты карты
    const mapX = (worldX - origin.position.x) / resolution;
    const mapY = (worldY - origin.position.y) / resolution;

    // Проверяем, что точка находится в пределах карты
    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) continue;

    // Преобразуем в пиксели на холсте
    const pixelX = offsetX + mapX * scale;
    const pixelY = offsetY + (height - mapY) * scale;
    
    // Рисуем вектор от робота к точке
    const robotMapX = (robotPosition.x - origin.position.x) / resolution;
    const robotMapY = (robotPosition.y - origin.position.y) / resolution;
    const robotPixelX = offsetX + robotMapX * scale;
    const robotPixelY = offsetY + (height - robotMapY) * scale;

    lidarCtx.beginPath();
    lidarCtx.moveTo(robotPixelX, robotPixelY);
    lidarCtx.lineTo(pixelX, pixelY);
    lidarCtx.stroke();
  }
  
  // Основная отрисовка точек лидара
  lidarCtx.fillStyle = "rgba(0, 255, 0, 0.7)";
  
  for (let i = 0; i < angleCount; i++) {
    const range = ranges[i];
    if (range === undefined || range === null || range === Infinity || 
        range < (scan?.range_min || 0) || range > (scan?.range_max || Infinity)) continue;

    // Поворот на 90 градусов против часовой стрелки (как в Python-примере)
    const angle = angle_min + i * angle_increment + Math.PI/2;
    
    // Преобразуем в координаты относительно робота
    const x = range * Math.cos(angle);
    const y = range * Math.sin(angle);

    // Мировые координаты точки лидара
    const worldX = robotPosition.x + x;
    const worldY = robotPosition.y + y;

    // Правильное преобразование в координаты карты
    const mapX = (worldX - origin.position.x) / resolution;
    const mapY = (worldY - origin.position.y) / resolution;

    // Проверяем, что точка находится в пределах карты
    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) continue;

    // Преобразуем в пиксели на холсте
    // Важно: в canvas Y растет вниз, поэтому инвертируем Y относительно карты
    // В ROS начало карты (0,0) находится в левом нижнем углу
    // В canvas начало (0,0) находится в левом верхнем углу
    const pixelX = offsetX + mapX * scale;
    const pixelY = offsetY + (height - mapY) * scale;

    // Рисуем точку
    lidarCtx.fillRect(pixelX, pixelY, 2, 2);
  }
  
  // Отрисовка позиции робота
  const robotMapX = (robotPosition.x - origin.position.x) / resolution;
  const robotMapY = (robotPosition.y - origin.position.y) / resolution;
  
  const robotPixelX = offsetX + robotMapX * scale;
  const robotPixelY = offsetY + (height - robotMapY) * scale;
  
  // Отрисовка робота как круга
  lidarCtx.beginPath();
  lidarCtx.arc(robotPixelX, robotPixelY, 5, 0, Math.PI * 2);
  lidarCtx.fillStyle = "rgba(255, 0, 0, 0.7)";
  lidarCtx.fill();
  
  // Отрисовка направления робота
  const directionX = robotPixelX + 10 * Math.cos(robotPosition.theta);
  const directionY = robotPixelY - 10 * Math.sin(robotPosition.theta); // Инвертируем Y для canvas
  
  lidarCtx.beginPath();
  lidarCtx.moveTo(robotPixelX, robotPixelY);
  lidarCtx.lineTo(directionX, directionY);
  lidarCtx.strokeStyle = "rgba(0, 0, 255, 0.7)";
  lidarCtx.lineWidth = 2;
  lidarCtx.stroke();
  
  // Отрисовка текущей цели (если есть)
  if (currentGoal) {
    const goalMapX = (currentGoal.x - origin.position.x) / resolution;
    const goalMapY = (currentGoal.y - origin.position.y) / resolution;
    
    const goalPixelX = offsetX + goalMapX * scale;
    const goalPixelY = offsetY + (height - goalMapY) * scale;
    
    // Отрисовка цели как квадрата
    lidarCtx.fillStyle = "rgba(255, 0, 0, 0.5)";
    lidarCtx.fillRect(goalPixelX - 4, goalPixelY - 4, 8, 8);
    
    // Линия от робота к цели
    lidarCtx.beginPath();
    lidarCtx.moveTo(robotPixelX, robotPixelY);
    lidarCtx.lineTo(goalPixelX, goalPixelY);
    lidarCtx.strokeStyle = "rgba(255, 0, 0, 0.3)";
    lidarCtx.lineWidth = 1;
    lidarCtx.stroke();
  }
  
  // Отрисовка траектории из /plan
  if (currentPlan && currentPlan.length > 1) {
    lidarCtx.beginPath();
    lidarCtx.moveTo(
      offsetX + ((currentPlan[0].x - origin.position.x) / resolution) * scale,
      offsetY + (height - (currentPlan[0].y - origin.position.y) / resolution) * scale
    );
    
    for (let i = 1; i < currentPlan.length; i++) {
      const px = offsetX + ((currentPlan[i].x - origin.position.x) / resolution) * scale;
      const py = offsetY + (height - (currentPlan[i].y - origin.position.y) / resolution) * scale;
      
      lidarCtx.lineTo(px, py);
    }
    
    lidarCtx.strokeStyle = "rgba(0, 0, 255, 0.7)";
    lidarCtx.lineWidth = 2;
    lidarCtx.stroke();
    
    // Отрисовка текущей точки траектории
    if (currentPlan.length > 0) {
      const currentGoalX = currentPlan[0].x;
      const currentGoalY = currentPlan[0].y;
      
      const goalMapX = (currentGoalX - origin.position.x) / resolution;
      const goalMapY = (currentGoalY - origin.position.y) / resolution;
      
      const goalPixelX = offsetX + goalMapX * scale;
      const goalPixelY = offsetY + (height - goalMapY) * scale;
      
      // Отрисовка текущей цели как квадрата
      lidarCtx.fillStyle = "rgba(0, 255, 0, 0.7)";
      lidarCtx.fillRect(goalPixelX - 4, goalPixelY - 4, 8, 8);
    }
  }
}

// Сохраняем последний лидарный скан для перерисовки при изменении позиции робота
let lastLidarScan: any = null;

// Сохраняем текущую цель для отрисовки
let currentGoal: { x: number, y: number } | null = null;

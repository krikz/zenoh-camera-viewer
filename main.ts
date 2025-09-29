import { parseCDRBytes } from "@mono424/cdr-ts";
import { OccupancyGrid, occupancyGridSchema } from "./map";
import { MessageWriter } from "@lichtblick/omgidl-serialization";
import {
  navigateToPoseDefinition,
  NavigateToPoseGoal,
  Twist,
  twistDefinition,
} from "./idl";

// Создаём writer один раз (можно кэшировать)
const goalWriter = new MessageWriter(
  "nav2_msgs::NavigateToPose_Goal",
  navigateToPoseDefinition,
  { kind: 0x01 } // XCDR не требуется
);

// Создаём writer для Twist (добавляем этот код)
const twistWriter = new MessageWriter(
  "geometry_msgs::Twist",
  twistDefinition,
  { kind: 0x01 } // XCDR не требуется
);

const SPEED_STEP = 0.05; // Increment/decrement step for speed
const ROTATION_STEP = 1 * (Math.PI / 180); // Increment/decrement step for direction in radians (1 degree)

// Add these new variables to track state
let currentLinearSpeed = 0.0;
let currentAngularSpeed = 0.0; // Represents the desired change in orientation over time
let targetDirection = 0.0; // Current target direction in radians

function toggleHelp() {
  const overlay = document.getElementById("helpOverlay");
  if (overlay) {
    overlay.style.display = overlay.style.display === "none" ? "flex" : "none";
  }
}

document.addEventListener("keydown", handleKeyDown);

function handleKeyDown(event: KeyboardEvent) {
  let speedChanged = false;
  let directionChanged = false;

  switch (event.key) {
    case "ArrowUp":
      event.preventDefault(); // Prevent page scrolling
      currentLinearSpeed = Math.min(
        MAX_LINEAR_SPEED,
        currentLinearSpeed + SPEED_STEP
      );
      speedChanged = true;
      console.log(`Speed increased to: ${currentLinearSpeed.toFixed(2)}`); // Optional: Log for debugging
      break;
    case "ArrowDown":
      event.preventDefault(); // Prevent page scrolling
      currentLinearSpeed = Math.max(
        -MAX_LINEAR_SPEED,
        currentLinearSpeed - SPEED_STEP
      ); // Allow reverse
      speedChanged = true;
      console.log(`Speed decreased to: ${currentLinearSpeed.toFixed(2)}`); // Optional: Log for debugging
      break;
    case "ArrowLeft":
      event.preventDefault();
      targetDirection += ROTATION_STEP;
      directionChanged = true;
      console.log(
        `Direction turned left to: ${(
          (targetDirection * 180) /
          Math.PI
        ).toFixed(1)}°`
      ); // Optional: Log for debugging
      break;
    case "ArrowRight":
      event.preventDefault();
      targetDirection -= ROTATION_STEP; // Negative because right turn is negative rotation
      directionChanged = true;
      console.log(
        `Direction turned right to: ${(
          (targetDirection * 180) /
          Math.PI
        ).toFixed(1)}°`
      ); // Optional: Log for debugging
      break;
    case " ":
      event.preventDefault();
      currentLinearSpeed = 0.0;
      currentAngularSpeed = 0.0; // Stop rotation as well
      speedChanged = true;
      directionChanged = true; // Technically stops changing direction too
      console.log(`Full stop requested.`); // Optional: Log for debugging
      break;
    case "Escape":
      toggleHelp(); // Close help if open
      break;
    default:
      // Ignore other keys
      break;
  }

  // If speed or direction changed, send the updated command
  if (speedChanged || directionChanged) {
    // Calculate angular speed based on target direction vs current robot theta
    // This is a simplified approach - you might want a more sophisticated controller
    // For now, let's assume the angular speed is proportional to the difference between
    // the target direction and the current robot orientation (robotPosition.theta)
    // and the desired change in speed.
    // A simple proportional controller:
    const directionError = targetDirection - robotPosition.theta;
    // Use a small proportional gain (adjust as needed)
    const Kp = 1.0;
    // The angular speed command tries to correct the orientation error
    // and also accounts for the desired linear speed if turning.
    // For simplicity, if linear speed is non-zero and direction changed, apply angular correction.
    if (directionChanged && currentLinearSpeed !== 0) {
      currentAngularSpeed = Kp * directionError;
      // Optionally limit angular speed
      currentAngularSpeed = Math.max(
        -MAX_ANGULAR_SPEED,
        Math.min(MAX_ANGULAR_SPEED, currentAngularSpeed)
      );
    } else if (speedChanged && !directionChanged) {
      // If only speed changed and direction is the same, angular speed remains based on previous error
      // or we could set it to 0 if no turning is intended, depending on desired behavior.
      // For now, let's assume no intentional turn means keep angular speed as calculated.
      // If you want to stop turning when only speed changes, uncomment the line below:
      // if (Math.abs(currentAngularSpeed) < 0.01) currentAngularSpeed = 0; // Small threshold to stop
    }
    // If both changed, the new angular speed based on error is already calculated above.

    pubTwist(currentLinearSpeed, currentAngularSpeed);
  }
}

// Find the showHelpBtn and attach the toggleHelp function
const showHelpBtn = document.getElementById("showHelpBtn") as HTMLButtonElement;
if (showHelpBtn) {
  showHelpBtn.onclick = toggleHelp;
}

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
// const ZENOH_REST_BASE = "https://zenoh.robbox.online";
const ZENOH_REST_BASE = "http://localhost:8000";

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
                      x: {
                        index: 0,
                        value: { type: "float", len: 64, format: "number" },
                      },
                      y: {
                        index: 1,
                        value: { type: "float", len: 64, format: "number" },
                      },
                      z: {
                        index: 2,
                        value: { type: "float", len: 64, format: "number" },
                      },
                    },
                  },
                },
                orientation: {
                  index: 1,
                  value: {
                    type: "dictionary",
                    items: {
                      x: {
                        index: 0,
                        value: { type: "float", len: 64, format: "number" },
                      },
                      y: {
                        index: 1,
                        value: { type: "float", len: 64, format: "number" },
                      },
                      z: {
                        index: 2,
                        value: { type: "float", len: 64, format: "number" },
                      },
                      w: {
                        index: 3,
                        value: { type: "float", len: 64, format: "number" },
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
                        x: {
                          index: 0,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        y: {
                          index: 1,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        z: {
                          index: 2,
                          value: { type: "float", len: 64, format: "number" },
                        },
                      },
                    },
                  },
                  rotation: {
                    index: 1,
                    value: {
                      type: "dictionary",
                      items: {
                        x: {
                          index: 0,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        y: {
                          index: 1,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        z: {
                          index: 2,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        w: {
                          index: 3,
                          value: { type: "float", len: 64, format: "number" },
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
                        x: {
                          index: 0,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        y: {
                          index: 1,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        z: {
                          index: 2,
                          value: { type: "float", len: 64, format: "number" },
                        },
                      },
                    },
                  },
                  orientation: {
                    index: 1,
                    value: {
                      type: "dictionary",
                      items: {
                        x: {
                          index: 0,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        y: {
                          index: 1,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        z: {
                          index: 2,
                          value: { type: "float", len: 64, format: "number" },
                        },
                        w: {
                          index: 3,
                          value: { type: "float", len: 64, format: "number" },
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
  },
} as const;

// Текущая карта для отрисовки лидара
let currentMap: OccupancyGrid | null = null;

// Текущая позиция робота в системе координат карты (map)
let robotPosition = { x: 0, y: 0, theta: 0 };

// Текущая траектория из /plan
let currentPlan: any[] = [];

// Трансформация от map к odom
let mapToOdom = {
  translation: { x: 0, y: 0, z: 0 },
  rotation: { x: 0, y: 0, z: 0, w: 1 },
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
        setupUnifiedFeed(robotName);
      } else {
        cleanupRobotFeeds();
        statusEl.textContent = "Выберите робота";
      }
    };

    // Автоматическая загрузка при наличии выбора
    if (robotSelect.value) {
      setupUnifiedFeed(robotSelect.value);
    }
  } catch (err) {
    console.error("[REST] Ошибка получения списка роботов:", err);
    robotSelect.innerHTML = `<option>Ошибка: ${
      err instanceof Error ? err.message : "Неизвестная ошибка"
    }</option>`;
  }
}

let robotEventSource: EventSource | null = null;

function setupUnifiedFeed(robotName: string) {
  cleanupRobotFeeds(); // закрываем старые

  const keyExpr = `robots/${robotName}/**`;
  const url = `${ZENOH_REST_BASE}/${keyExpr}`;

  robotEventSource = new EventSource(url);

  robotEventSource.addEventListener("open", () => {
    console.log(`[SSE] Unified feed started for ${robotName}`);
    statusEl.textContent = `📡 Получение данных с ${robotName}...`;
  });

  robotEventSource.addEventListener("error", (err) => {
    console.error("[SSE Unified] Ошибка:", err);
    statusEl.textContent = "⚠️ Ошибка SSE";
    if (robotEventSource?.readyState === EventSource.CLOSED) {
      robotEventSource = null;
    }
  });

  robotEventSource.addEventListener("PUT", handleRobot);
}

function handleRobot(event: MessageEvent) {
  try {
    const sample = JSON.parse(event.data) as { key: string; value: string };

    // Логируем (опционально)
    // console.log("[SSE] Received:", sample.key);

    if (
      sample.key.endsWith("/robot_cam")
    ) {
      handleCameraEvent(sample);
    } else if (sample.key.endsWith("/map")) {
      handleMapEvent(sample);
    } else if (sample.key.endsWith("/scan")) {
      handleLidarEvent(sample);
    } else if (
      //sample.key.includes("/odom") ||
      sample.key.endsWith("/diff_drive_base_controller/odom")
    ) {
      handleOdometryEvent(sample);
    } else if (sample.key.endsWith("/tf")) {
      handleTfEvent(sample);
    } else if (sample.key.endsWith("/plan")) {
      handlePlanEvent(sample);
    } else {
      // console.debug("Unknown topic:", sample.key);
    }
  } catch (err) {
    console.error("[SSE Unified] Parse error:", err);
  }
}

function cleanupRobotFeeds() {
  // Закрываем все EventSource
  if (robotEventSource) {
    robotEventSource.removeEventListener("PUT", handleRobot);
    robotEventSource.close();
    robotEventSource = null;
  }
  // Очищаем лидарный холст при смене робота
  if (lidarCtx) {
    lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
  }

  // Очищаем текущую траекторию
  currentPlan = [];
}

// Обработчики событий
function handleCameraEvent(sample: { key: string; value: string }) {
  try {
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

function handleMapEvent(sample: { key: string; value: string }) {
  try {
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

function handleLidarEvent(sample: { key: string; value: string }) {
  try {
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

function handleOdometryEvent(sample: { key: string; value: string }) {
  try {
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
    const odomTheta = Math.atan2(
      2 * (q.w * q.z + q.x * q.y),
      1 - 2 * (q.y * q.y + q.z * q.z)
    );

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

function handleTfEvent(sample: { key: string; value: string }) {
  try {
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
      if (
        transform.header.frame_id === "map" &&
        transform.child_frame_id === "odom"
      ) {
        mapToOdom = {
          translation: transform.transform.translation,
          rotation: transform.transform.rotation,
        };
        break;
      }
    }
  } catch (err) {
    console.error("[SSE TF] Обработка падения:", err);
  }
}

function handlePlanEvent(sample: { key: string; value: string }) {
  try {
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
      y: pose.pose.position.y,
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
  // Трансформация из odom в map (mapToOdom преобразует точки из odom в map)
  const { x: tx, y: ty } = mapToOdom.translation;
  const { x: qx, y: qy, z: qz, w: qw } = mapToOdom.rotation;

  // Вычисляем угол поворота трансформации odom -> map
  const odomToMapTheta = Math.atan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy * qy + qz * qz)
  );

  // Преобразуем точку из системы odom в систему map
  const cosTheta = Math.cos(odomToMapTheta);
  const sinTheta = Math.sin(odomToMapTheta);

  // Правильное преобразование: сначала поворот, затем смещение
  const mapX = tx + cosTheta * odomX - sinTheta * odomY;
  const mapY = ty + sinTheta * odomX + cosTheta * odomY;

  // Угол робота в системе map (складываем углы)
  const mapTheta = odomTheta + odomToMapTheta;

  return { x: mapX, y: mapY, theta: mapTheta };
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
      const robotCellX = Math.round(
        (robotPosition.x - map.info.origin.position.x) / map.info.resolution
      );
      const robotCellY = Math.round(
        (robotPosition.y - map.info.origin.position.y) / map.info.resolution
      );

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

// Глобальные переменные для управления камерой
let isDragging = false;
let isResizing = false;
let dragStartX = 0;
let dragStartY = 0;
let cameraStartX = 0;
let cameraStartY = 0;
let cameraStartWidth = 0;
let cameraStartHeight = 0;
let lastCameraPosition = { x: 20, y: 20, width: 240 };

// Инициализация управления камерой
function initCameraControl() {
  const cameraOverlay = document.querySelector(
    ".camera-overlay"
  ) as HTMLElement;
  const resizeHandle = cameraOverlay.querySelector(
    ".resize-handle"
  ) as HTMLElement;

  if (!cameraOverlay || !resizeHandle) {
    console.error("Элементы управления камерой не найдены");
    return;
  }

  // Загрузка сохраненного положения камеры из localStorage
  loadCameraPosition();

  // Обработчики событий для перемещения
  cameraOverlay.addEventListener("mousedown", startDrag);
  cameraOverlay.addEventListener("touchstart", (e) => {
    e.preventDefault();
    startDrag({
      clientX: e.touches[0].clientX,
      clientY: e.touches[0].clientY,
      button: 0,
    } as MouseEvent);
  });

  // Обработчики событий для изменения размера
  resizeHandle.addEventListener("mousedown", startResize);
  resizeHandle.addEventListener("touchstart", (e) => {
    e.preventDefault();
    startResize({
      clientX: e.touches[0].clientX,
      clientY: e.touches[0].clientY,
    } as MouseEvent);
  });

  // Общие обработчики для mousemove и mouseup
  document.addEventListener("mousemove", drag);
  document.addEventListener(
    "touchmove",
    (e) => {
      e.preventDefault();
      drag({
        clientX: e.touches[0].clientX,
        clientY: e.touches[0].clientY,
      } as MouseEvent);
    },
    { passive: false }
  );

  document.addEventListener("mouseup", endDrag);
  document.addEventListener("touchend", endDrag);

  // Функция для начала перетаскивания
  function startDrag(e: MouseEvent) {
    if (e.button !== 0) return; // Только левая кнопка мыши
    isDragging = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    cameraStartX = cameraOverlay.offsetLeft;
    cameraStartY = cameraOverlay.offsetTop;
  }

  // Функция для начала изменения размера
  function startResize(e: MouseEvent) {
    e.stopPropagation(); // Предотвращаем начало перетаскивания
    isResizing = true;
    dragStartX = e.clientX;
    dragStartY = e.clientY;
    cameraStartWidth = cameraOverlay.offsetWidth;
    cameraStartHeight = cameraOverlay.offsetHeight;
  }

  // Основная функция перемещения
  function drag(e: MouseEvent) {
    if (!isDragging && !isResizing) return;

    const mainArea = document.querySelector(".main-area") as HTMLElement;
    if (!mainArea) return;

    const rect = mainArea.getBoundingClientRect();

    if (isDragging) {
      // Вычисляем новую позицию
      let newX = cameraStartX + (e.clientX - dragStartX);
      let newY = cameraStartY + (e.clientY - dragStartY);

      // Ограничиваем перемещение внутри main-area
      newX = Math.max(
        0,
        Math.min(newX, rect.width - cameraOverlay.offsetWidth)
      );
      newY = Math.max(
        0,
        Math.min(newY, rect.height - cameraOverlay.offsetHeight)
      );

      // Применяем новую позицию
      cameraOverlay.style.left = `${newX}px`;
      cameraOverlay.style.top = `${newY}px`;

      // Сохраняем позицию
      lastCameraPosition.x = newX;
      lastCameraPosition.y = newY;
      saveCameraPosition();
    }

    if (isResizing) {
      // Вычисляем новый размер
      let newWidth = cameraStartWidth + (e.clientX - dragStartX);
      let newHeight = cameraStartHeight + (e.clientY - dragStartY);

      // Ограничиваем минимальный размер
      newWidth = Math.max(100, newWidth);
      newHeight = Math.max(75, newHeight);

      // Применяем новый размер
      cameraOverlay.style.width = `${newWidth}px`;
      // Сохраняем соотношение сторон 4:3
      cameraOverlay.style.height = `${newWidth * 0.75}px`;

      // Сохраняем размер
      lastCameraPosition.width = newWidth;
      saveCameraPosition();
    }
  }

  // Завершение перетаскивания/изменения размера
  function endDrag() {
    isDragging = false;
    isResizing = false;
  }

  // Сохранение позиции камеры в localStorage
  function saveCameraPosition() {
    try {
      localStorage.setItem(
        "cameraPosition",
        JSON.stringify(lastCameraPosition)
      );
    } catch (e) {
      console.warn("Не удалось сохранить позицию камеры:", e);
    }
  }

  // Загрузка позиции камеры из localStorage
  function loadCameraPosition() {
    try {
      const saved = localStorage.getItem("cameraPosition");
      if (saved) {
        const pos = JSON.parse(saved);
        lastCameraPosition = pos;

        // Применяем сохраненные значения
        cameraOverlay.style.left = `${pos.x}px`;
        cameraOverlay.style.top = `${pos.y}px`;
        cameraOverlay.style.width = `${pos.width}px`;
        cameraOverlay.style.height = `${pos.width * 0.75}px`;
      }
    } catch (e) {
      console.warn("Не удалось загрузить позицию камеры:", e);
    }
  }
}

// Вызовите эту функцию после инициализации остальных компонентов
initCameraControl();

function resizeCanvases() {
  // Карта — занимает всё свободное место
  const rect = mapCanvas.parentElement!.getBoundingClientRect();
  mapCanvas.width = rect.width;
  mapCanvas.height = rect.height;
  lidarCanvas.width = rect.width;
  lidarCanvas.height = rect.height;

  // Сохраняем размеры камеры из последней позиции
  const cameraOverlay = document.querySelector(
    ".camera-overlay"
  ) as HTMLElement;
  if (cameraOverlay && lastCameraPosition) {
    // Обновляем размеры canvas камеры в соответствии с размером обертки
    const camWidth = lastCameraPosition.width;
    cameraCanvas.width = camWidth;
    cameraCanvas.height = camWidth * 0.75; // Сохраняем соотношение 4:3
  }
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
  lidarCtx.strokeStyle = "rgba(255, 165, 0, 0.9)";
  lidarCtx.lineWidth = 1;

  for (let i = 0; i < angleCount; i += 1) {
    const range = ranges[i];
    if (
      range === undefined ||
      range === null ||
      range === Infinity ||
      range < (scan?.range_min || 0) ||
      range > (scan?.range_max || Infinity)
    )
      continue;

    // Угол луча лидара относительно робота
    const beamAngle = angle_min + i * angle_increment;

    // Координаты точки лидара в ЛОКАЛЬНОЙ системе координат робота
    const x_local = range * Math.cos(beamAngle);
    const y_local = range * Math.sin(beamAngle);

    // ПРАВИЛЬНОЕ преобразование в МИРОВУЮ систему координат
    // Учитываем ориентацию робота
    const cosRobot = Math.cos(robotPosition.theta);
    const sinRobot = Math.sin(robotPosition.theta);

    const worldX = robotPosition.x + x_local * cosRobot - y_local * sinRobot;
    const worldY = robotPosition.y + x_local * sinRobot + y_local * cosRobot;

    // Правильное преобразование в координаты карты
    const mapX = (worldX - origin.position.x) / resolution;
    const mapY = (worldY - origin.position.y) / resolution;

    // Проверяем, что точка находится в пределах карты
    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) continue;

    // Преобразуем в пиксели на холсте
    const pixelX = offsetX + mapX * scale;
    const pixelY = offsetY + mapY * scale;

    // Рисуем вектор от робота к точке
    const robotMapX = (robotPosition.x - origin.position.x) / resolution;
    const robotMapY = (robotPosition.y - origin.position.y) / resolution;
    const robotPixelX = offsetX + robotMapX * scale;
    const robotPixelY = offsetY + robotMapY * scale;

    lidarCtx.beginPath();
    lidarCtx.moveTo(robotPixelX, robotPixelY);
    lidarCtx.lineTo(pixelX, pixelY);
    lidarCtx.stroke();
  }

  // Основная отрисовка точек лидара
  lidarCtx.fillStyle = "rgba(0, 255, 0, 0.7)";

  for (let i = 0; i < angleCount; i++) {
    const range = ranges[i];
    if (
      range === undefined ||
      range === null ||
      range === Infinity ||
      range < (scan?.range_min || 0) ||
      range > (scan?.range_max || Infinity)
    )
      continue;

    // Угол луча лидара
    const beamAngle = angle_min + i * angle_increment;

    // Координаты точки лидара в ЛОКАЛЬНОЙ системе координат робота
    const x_local = range * Math.cos(beamAngle);
    const y_local = range * Math.sin(beamAngle);

    // ПРАВИЛЬНОЕ преобразование в МИРОВУЮ систему координат
    // Учитываем ориентацию робота
    const cosRobot = Math.cos(robotPosition.theta);
    const sinRobot = Math.sin(robotPosition.theta);

    const worldX = robotPosition.x + x_local * cosRobot - y_local * sinRobot;
    const worldY = robotPosition.y + x_local * sinRobot + y_local * cosRobot;

    // Преобразуем в координаты карты
    const mapX = (worldX - origin.position.x) / resolution;
    const mapY = (worldY - origin.position.y) / resolution;

    // Проверяем, что точка находится в пределах карты
    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) continue;

    // Преобразуем в пиксели на холсте
    // Важно: в canvas Y растет вниз, поэтому инвертируем Y относительно карты
    // В ROS начало карты (0,0) находится в левом нижнем углу
    // В canvas начало (0,0) находится в левом верхнем углу
    const pixelX = offsetX + mapX * scale;
    const pixelY = offsetY + mapY * scale;

    // Рисуем точку
    lidarCtx.fillRect(pixelX, pixelY, 2, 2);
  }

  // Отрисовка позиции робота
  const robotMapX = (robotPosition.x - origin.position.x) / resolution;
  const robotMapY = (robotPosition.y - origin.position.y) / resolution;

  const robotPixelX = offsetX + robotMapX * scale;
  const robotPixelY = offsetY + robotMapY * scale;

  // Отрисовка робота как круга
  lidarCtx.beginPath();
  lidarCtx.arc(robotPixelX, robotPixelY, 5, 0, Math.PI * 2);
  lidarCtx.fillStyle = "rgba(255, 0, 0, 0.7)";
  lidarCtx.fill();

  // Отрисовка направления робота
  const directionX = robotPixelX + 30 * Math.cos(robotPosition.theta);
  const directionY = robotPixelY + 30 * Math.sin(robotPosition.theta);

  lidarCtx.beginPath();
  lidarCtx.moveTo(robotPixelX, robotPixelY);
  lidarCtx.lineTo(directionX, directionY);
  lidarCtx.strokeStyle = "rgba(0, 0, 255, 0.7)";
  lidarCtx.lineWidth = 4;
  lidarCtx.stroke();

  // Отрисовка текущей цели (если есть)
  if (currentGoal) {
    const goalMapX = (currentGoal.x - origin.position.x) / resolution;
    const goalMapY = (currentGoal.y - origin.position.y) / resolution;

    const goalPixelX = offsetX + goalMapX * scale;
    const goalPixelY = offsetY + goalMapY * scale;

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
      offsetY + ((currentPlan[0].y - origin.position.y) / resolution) * scale
    );

    for (let i = 1; i < currentPlan.length; i++) {
      const px =
        offsetX + ((currentPlan[i].x - origin.position.x) / resolution) * scale;
      const py =
        offsetY + ((currentPlan[i].y - origin.position.y) / resolution) * scale;

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
      const goalPixelY = offsetY + goalMapY * scale;

      // Отрисовка текущей цели как квадрата
      lidarCtx.fillStyle = "rgba(0, 255, 0, 0.7)";
      lidarCtx.fillRect(goalPixelX - 4, goalPixelY - 4, 8, 8);
    }
  }
}

// Добавьте эту функцию в ваш main.ts
async function pubTwist() {
  // No parameters needed now
  const robotName = robotSelect.value;
  if (!robotName) {
    console.warn("Not selected robot for sending command");
    return;
  }
  try {
    // Use the global variables for speed
    const twist: Twist = {
      linear: { x: currentLinearSpeed, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: currentAngularSpeed },
    };
    console.log(
      `Sending Twist: linear=${twist.linear.x.toFixed(
        2
      )}, angular=${twist.angular.z.toFixed(2)}`
    ); // Log command

    const cdrBytes: Uint8Array = twistWriter.writeMessage(twist);
    const key = `robots/${robotName}/cmd_vel`;
    const url = `${ZENOH_REST_BASE}/${key}`;
    const response = await fetch(url, {
      method: "PUT",
      headers: {
        "Content-Type": "application/octet-stream",
      },
      body: cdrBytes,
    });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}: ${await response.text()}`);
    }
    console.log(
      `✅ Velocity command sent: linear=${twist.linear.x.toFixed(
        2
      )}, angular=${twist.angular.z.toFixed(2)}`
    );
  } catch (err) {
    console.error("❌ Error sending velocity command:", err);
    statusEl.textContent = `⚠️ Error sending command: ${
      err instanceof Error ? err.message : "Unknown error"
    }`;
  }
}
// Сохраняем последний лидарный скан для перерисовки при изменении позиции робота
let lastLidarScan: any = null;

// Сохраняем текущую цель для отрисовки
let currentGoal: { x: number; y: number } | null = null;

// Глобальные переменные для геймпад
// Индексы осей для Radiomaster (по вашим данным)
const PITCH_AXIS = 1; // Движение вперед/назад (2-я ось, индекс 1)
const YAW_AXIS = 3; // Поворот влево/вправо (4-я ось, индекс 3)
let gamepadConnected = false;
let gamepadInterval: number | null = null;
const MAX_LINEAR_SPEED = 0.5; // Максимальная линейная скорость
const MAX_ANGULAR_SPEED = 1.0; // Максимальная угловая скорость
const DEADZONE = 0.1; // Мертвая зона стиков

// Инициализация геймпада
function initGamepadControl() {
  const gamepadBtn = document.getElementById("gamepadBtn") as HTMLButtonElement;
  const gamepadOverlay = document.getElementById("gamepadOverlay")!;
  const pitchIndicator = document.getElementById("pitchIndicator")!;
  const yawIndicator = document.getElementById("yawIndicator")!;

  if (!("getGamepads" in navigator)) {
    gamepadBtn.disabled = true;
    gamepadBtn.title = "Браузер не поддерживает Gamepad API";
    gamepadBtn.textContent = "🎮 Gamepad недоступен";
    return;
  }

  // Обработчик подключения геймпада
  window.addEventListener("gamepadconnected", (e: GamepadEvent) => {
    if (!gamepadConnected) {
      connectGamepad();
    }
  });

  // Обработчик отключения геймпада
  window.addEventListener("gamepaddisconnected", () => {
    if (gamepadConnected) {
      disconnectGamepad();
    }
  });

  // Обработчик нажатия кнопки
  gamepadBtn.addEventListener("click", () => {
    if (gamepadConnected) {
      disconnectGamepad();
    } else {
      connectGamepad();
    }
  });

  function connectGamepad() {
    const gamepads = navigator.getGamepads();
    let gamepadFound = false;

    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        gamepadFound = true;
        break;
      }
    }

    if (!gamepadFound) {
      statusEl.textContent =
        "⚠️ Нажмите любую кнопку на геймпаде для активации";
      return;
    }

    // Настройка интервала опроса
    if (gamepadInterval) {
      clearInterval(gamepadInterval);
    }

    gamepadInterval = window.setInterval(readGamepad, 50); // 20 Гц
    gamepadConnected = true;

    // Обновление UI
    gamepadBtn.classList.add("connected");
    gamepadBtn.textContent = "⏹ Disconnect Gamepad";
    gamepadOverlay.style.display = "block";
    robotVisual.style.display = "block";
    statusEl.textContent = "🎮 Геймпад подключен";
  }

  function disconnectGamepad() {
    if (gamepadInterval) {
      clearInterval(gamepadInterval);
      gamepadInterval = null;
    }

    // Отправка команды остановки
    pubTwist(0, 0);

    // Обновление UI
    gamepadConnected = false;
    gamepadBtn.classList.remove("connected");
    gamepadBtn.textContent = "🎮 Connect Gamepad";
    gamepadOverlay.style.display = "none";
    robotVisual.style.display = "none";
    statusEl.textContent = "Геймпад отключен";
  }

  function readGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepads.length === 0 || !gamepads[0]) {
      disconnectGamepad();
      return;
    }

    const gamepad = gamepads[0];

    // Получаем данные с нужных осей (по вашим данным)
    const pitch = gamepad.axes[PITCH_AXIS]; // Движение вперед/назад (индекс 1)
    const yaw = gamepad.axes[YAW_AXIS]; // Поворот влево/вправо (индекс 3)

    // Применяем мертвую зону
    const pitchValue = Math.abs(pitch) > DEADZONE ? pitch : 0;
    const yawValue = Math.abs(yaw) > DEADZONE ? yaw : 0;

    // Обновляем индикаторы
    updateGamepadVisualization(pitchValue, yawValue);

    // Если оба значения в мертвой зоне, отправляем 0 (остановка)
    if (pitchValue === 0 && yawValue === 0) {
      pubTwist(0, 0);
      return;
    }

    // Преобразуем в линейную и угловую скорость
    const linear = pitchValue * MAX_LINEAR_SPEED;
    const angular = -yawValue * MAX_ANGULAR_SPEED;
    // Обновляем визуализацию робота
    updateRobotVisualization(linear, angular);
    // Отправляем команду через существующую функцию
    pubTwist(linear, angular);
  }

  function updateGamepadVisualization(pitch: number, yaw: number) {
    // Для pitch: -1 (назад) -> 0%, 0 (нейтраль) -> 50%, 1 (вперед) -> 100%
    const pitchPercent = (pitch + 1) * 50;
    pitchIndicator.style.width = `${pitchPercent}%`;
    pitchIndicator.style.backgroundColor = "#f44336";

    // Для yaw: -1 (вправо) -> 0%, 0 (нейтраль) -> 50%, 1 (влево) -> 100%
    const yawPercent = (yaw + 1) * 50;
    yawIndicator.style.width = `${yawPercent}%`;
    yawIndicator.style.backgroundColor = "#f44336";
  }
}

// Элементы визуализации робота
const robotVisual = document.getElementById("robotVisual")!;
const robotCanvas = document.getElementById(
  "robotVisualCanvas"
) as HTMLCanvasElement;
const robotCtx = robotCanvas.getContext("2d")!;

// Функция для отрисовки состояния робота
function updateRobotVisualization(linear: number, angular: number) {
  // Очистка холста
  robotCtx.clearRect(0, 0, robotCanvas.width, robotCanvas.height);

  // Параметры робота
  const robotWidth = 60; // Ширина корпуса
  const robotHeight = 80; // Высота корпуса
  const wheelWidth = 10; // Ширина колеса
  const wheelHeight = 20; // Высота колеса
  const wheelEdgeOffset = -5; // Отступ от края корпуса до края колеса

  // Позиция робота на холсте
  const robotX = robotCanvas.width / 2;
  const robotY = robotCanvas.height / 2;

  // Рисуем корпус робота
  robotCtx.fillStyle = "#333";
  robotCtx.fillRect(
    robotX - robotWidth / 2,
    robotY - robotHeight / 2,
    robotWidth,
    robotHeight
  );

  // Рисуем колеса
  robotCtx.fillStyle = "#555";

  // Позиции центров колес (согласно требованиям:
  // колеса расположены на 5 пикселей внутри корпуса,
  // центр колеса на расстоянии 15 пикселей от края корпуса)
  const wheelCenterOffset = 15; // Отступ от края корпуса до центра колеса
  const leftWheelCenterX = robotX - robotWidth / 2 + wheelEdgeOffset;
  const rightWheelCenterX = robotX + robotWidth / 2 - wheelEdgeOffset;
  const topWheelCenterY = robotY - robotHeight / 2 + wheelCenterOffset;
  const bottomWheelCenterY = robotY + robotHeight / 2 - wheelCenterOffset;

  // Левое верхнее колесо
  robotCtx.fillRect(
    leftWheelCenterX - wheelWidth / 2,
    topWheelCenterY - wheelHeight / 2,
    wheelWidth,
    wheelHeight
  );
  // Левое нижнее колесо
  robotCtx.fillRect(
    leftWheelCenterX - wheelWidth / 2,
    bottomWheelCenterY - wheelHeight / 2,
    wheelWidth,
    wheelHeight
  );
  // Правое верхнее колесо
  robotCtx.fillRect(
    rightWheelCenterX - wheelWidth / 2,
    topWheelCenterY - wheelHeight / 2,
    wheelWidth,
    wheelHeight
  );
  // Правое нижнее колесо
  robotCtx.fillRect(
    rightWheelCenterX - wheelWidth / 2,
    bottomWheelCenterY - wheelHeight / 2,
    wheelWidth,
    wheelHeight
  );

  // Вычисляем скорости колес
  const maxSpeed = 1.0;
  const leftSpeed = linear - angular * (robotWidth / 100);
  const rightSpeed = linear + angular * (robotWidth / 100);

  // Ограничиваем скорости
  const normalizedLeftSpeed = Math.max(-1, Math.min(1, leftSpeed / maxSpeed));
  const normalizedRightSpeed = Math.max(-1, Math.min(1, rightSpeed / maxSpeed));

  // Определяем позицию для векторов (вертикально, от центра)
  const vectorLength = 40;
  const vectorYCenter = robotY;

  // Вектор для левой стороны (начинается от центра)
  const leftVectorX = robotX;
  const leftVectorYStart = vectorYCenter;
  const leftVectorYEnd = vectorYCenter - vectorLength * normalizedLeftSpeed;

  robotCtx.strokeStyle = normalizedLeftSpeed >= 0 ? "#4CAF50" : "#f44336";
  robotCtx.lineWidth = 3;
  robotCtx.beginPath();
  robotCtx.moveTo(leftVectorX - 15, leftVectorYStart);
  robotCtx.lineTo(leftVectorX - 15, leftVectorYEnd);
  robotCtx.stroke();

  // Стрелка для левого вектора
  robotCtx.fillStyle = robotCtx.strokeStyle;
  robotCtx.beginPath();
  if (normalizedLeftSpeed >= 0) {
    // Движение вверх (зеленый)
    robotCtx.moveTo(leftVectorX - 15, leftVectorYEnd);
    robotCtx.lineTo(leftVectorX - 15 - 4, leftVectorYEnd + 8);
    robotCtx.lineTo(leftVectorX - 15 + 4, leftVectorYEnd + 8);
  } else {
    // Движение вниз (красный)
    robotCtx.moveTo(leftVectorX - 15, leftVectorYEnd);
    robotCtx.lineTo(leftVectorX - 15 - 4, leftVectorYEnd - 8);
    robotCtx.lineTo(leftVectorX - 15 + 4, leftVectorYEnd - 8);
  }
  robotCtx.closePath();
  robotCtx.fill();

  // Вектор для правой стороны (начинается от центра)
  const rightVectorX = robotX;
  const rightVectorYStart = vectorYCenter;
  const rightVectorYEnd = vectorYCenter - vectorLength * normalizedRightSpeed;

  robotCtx.strokeStyle = normalizedRightSpeed >= 0 ? "#4CAF50" : "#f44336";
  robotCtx.lineWidth = 3;
  robotCtx.beginPath();
  robotCtx.moveTo(rightVectorX + 15, rightVectorYStart);
  robotCtx.lineTo(rightVectorX + 15, rightVectorYEnd);
  robotCtx.stroke();

  // Стрелка для правого вектора
  robotCtx.fillStyle = robotCtx.strokeStyle;
  robotCtx.beginPath();
  if (normalizedRightSpeed >= 0) {
    // Движение вверх (зеленый)
    robotCtx.moveTo(rightVectorX + 15, rightVectorYEnd);
    robotCtx.lineTo(rightVectorX + 15 - 4, rightVectorYEnd + 8);
    robotCtx.lineTo(rightVectorX + 15 + 4, rightVectorYEnd + 8);
  } else {
    // Движение вниз (красный)
    robotCtx.moveTo(rightVectorX + 15, rightVectorYEnd);
    robotCtx.lineTo(rightVectorX + 15 - 4, rightVectorYEnd - 8);
    robotCtx.lineTo(rightVectorX + 15 + 4, rightVectorYEnd - 8);
  }
  robotCtx.closePath();
  robotCtx.fill();
  /* 
  // Отображаем численные значения
  robotCtx.fillStyle = "#eee";
  robotCtx.font = "12px Arial";
  robotCtx.textAlign = "center";
  robotCtx.fillText(`L: ${leftSpeed.toFixed(2)}`, leftVectorX - 15, vectorYCenter + 25);
  robotCtx.fillText(`R: ${rightSpeed.toFixed(2)}`, rightVectorX + 15, vectorYCenter + 25);
  
  // Подписи
  robotCtx.fillText("Left", leftVectorX - 15, vectorYCenter - 50);
  robotCtx.fillText("Right", rightVectorX + 15, vectorYCenter - 50);
  */
}
// Вызовите эту функцию после инициализации остальных компонентов
initGamepadControl();

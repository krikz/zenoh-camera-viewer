// Map.ts
export interface Time {
  sec: number;
  nanosec: number;
}

export interface Pose {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number; w: number };
}

export interface MapMetaData {
  map_load_time: Time;
  resolution: number; // m/cell
  width: number;     // cells
  height: number;    // cells
  origin: Pose;
}

export interface OccupancyGrid {
  header: { stamp: Time; frame_id: string };
  info: MapMetaData;
  data: number[]; // int8[], -1 = unknown, 0 = free, 100 = occupied
}

import { parseCDRBytes } from '@mono424/cdr-ts';

export const occupancyGridSchema = {
  type: 'dictionary',
  items: {
    header: {
      index: 0,
      value: {
        type: 'dictionary',
        items: {
          stamp: {
            index: 0,
            value: {
              type: 'dictionary',
              items: {
                sec: { index: 0, value: { type: 'uint', len: 32, format: 'number' } },
                nanosec: { index: 1, value: { type: 'uint', len: 32, format: 'number' } }
              }
            }
          },
          frame_id: { index: 1, value: { type: 'string' } }
        }
      }
    },
    info: {
      index: 1,
      value: {
        type: 'dictionary',
        items: {
          map_load_time: {
            index: 0,
            value: {
              type: 'dictionary',
              items: {
                sec: { index: 0, value: { type: 'uint', len: 32, format: 'number' } },
                nanosec: { index: 1, value: { type: 'uint', len: 32, format: 'number' } }
              }
            }
          },
          resolution: { index: 1, value: { type: 'float', len: 32 } },
          width: { index: 2, value: { type: 'uint', len: 32, format: 'number' } },
          height: { index: 3, value: { type: 'uint', len: 32, format: 'number' } },
          origin: {
            index: 4,
            value: {
              type: 'dictionary',
              items: {
                position: {
                  index: 0,
                  value: {
                    type: 'dictionary',
                    items: {
                      x: { index: 0, value: { type: 'float', len: 64 } },
                      y: { index: 1, value: { type: 'float', len: 64 } },
                      z: { index: 2, value: { type: 'float', len: 64 } }
                    }
                  }
                },
                orientation: {
                  index: 1,
                  value: {
                    type: 'dictionary',
                    items: {
                      x: { index: 0, value: { type: 'float', len: 64 } },
                      y: { index: 1, value: { type: 'float', len: 64 } },
                      z: { index: 2, value: { type: 'float', len: 64 } },
                      w: { index: 3, value: { type: 'float', len: 64 } }
                    }
                  }
                }
              }
            }
          }
        }
      }
    },
    data: {
      index: 2,
      value: {
        type: 'sequence',
        itemSchema: { type: 'int', len: 8, format: 'number' }
      }
    }
  }
} as const;
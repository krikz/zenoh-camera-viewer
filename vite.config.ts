import { defineConfig } from 'vite';
import wasm from 'vite-plugin-wasm';
// import { resolve } from 'path'; // опционально

export default defineConfig({
  plugins: [
    wasm(), // <-- добавляем поддержку .wasm
  ],
  // Чтобы избежать предупреждений о top-level await
  worker: {
    format: 'es',
  },
  // Если используешь TypeScript
  resolve: {
    alias: {
      '@': new URL('./src', import.meta.url).pathname,
    },
  },
});
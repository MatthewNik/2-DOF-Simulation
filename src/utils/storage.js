import { DEFAULT_CONFIG, STORAGE_KEYS } from "../constants.js";

function safeParse(rawValue, fallback) {
  try {
    return rawValue ? JSON.parse(rawValue) : fallback;
  } catch {
    return fallback;
  }
}

export function loadSavedState() {
  const saved = safeParse(localStorage.getItem(STORAGE_KEYS.appState), null);
  if (!saved) {
    return DEFAULT_CONFIG;
  }

  return {
    ...DEFAULT_CONFIG,
    ...saved,
    jointLimits: {
      ...DEFAULT_CONFIG.jointLimits,
      ...(saved.jointLimits || {})
    },
    calibration: {
      ...DEFAULT_CONFIG.calibration,
      ...(saved.calibration || {})
    }
  };
}

export function saveState(config) {
  localStorage.setItem(STORAGE_KEYS.appState, JSON.stringify(config));
}

export function loadPresets() {
  return safeParse(localStorage.getItem(STORAGE_KEYS.presets), []);
}

export function savePresets(presets) {
  localStorage.setItem(STORAGE_KEYS.presets, JSON.stringify(presets));
}

import {
  createDefaultAppState,
  createDefaultLeg3Config,
  createDefaultPlanar2Config,
  DEFAULT_LEG3_MODULE_ORDER,
  DEFAULT_PLANAR2_MODULE_ORDER,
  STORAGE_KEYS
} from "../constants.js";

function safeParse(rawValue, fallback) {
  try {
    return rawValue ? JSON.parse(rawValue) : fallback;
  } catch {
    return fallback;
  }
}

function clone(value) {
  return typeof structuredClone === "function"
    ? structuredClone(value)
    : JSON.parse(JSON.stringify(value));
}

function mergePlainObject(defaultValue, savedValue) {
  if (!savedValue || typeof savedValue !== "object") {
    return clone(defaultValue);
  }

  const next = Array.isArray(defaultValue) ? [...defaultValue] : { ...defaultValue };
  for (const key of Object.keys(defaultValue)) {
    const defaultEntry = defaultValue[key];
    const savedEntry = savedValue[key];
    if (
      defaultEntry &&
      typeof defaultEntry === "object" &&
      !Array.isArray(defaultEntry)
    ) {
      next[key] = mergePlainObject(defaultEntry, savedEntry);
    } else {
      next[key] = savedEntry !== undefined ? savedEntry : defaultEntry;
    }
  }
  return next;
}

function migrateLegacyState(saved) {
  const defaultState = createDefaultAppState();
  if (!saved || typeof saved !== "object") {
    return defaultState;
  }

  if (saved.simulators && saved.activeMode) {
    const next = clone(defaultState);
    next.activeMode = saved.activeMode || next.activeMode;
    next.activeWorkspacePage = saved.activeWorkspacePage || next.activeWorkspacePage;
    next.theme = mergePlainObject(defaultState.theme, saved.theme);
    next.simulators.planar2.config = mergePlainObject(
      createDefaultPlanar2Config(),
      saved.simulators.planar2?.config
    );
    next.simulators.leg3.config = mergePlainObject(
      createDefaultLeg3Config(),
      saved.simulators.leg3?.config
    );
    next.simulators.planar2.ui.moduleOrder =
      saved.simulators.planar2?.ui?.moduleOrder || [...DEFAULT_PLANAR2_MODULE_ORDER];
    next.simulators.leg3.ui.moduleOrder =
      saved.simulators.leg3?.ui?.moduleOrder || [...DEFAULT_LEG3_MODULE_ORDER];
    next.simulators.planar2.trajectory = mergePlainObject(
      next.simulators.planar2.trajectory,
      saved.simulators.planar2?.trajectory
    );
    next.simulators.leg3.trajectory = mergePlainObject(
      next.simulators.leg3.trajectory,
      saved.simulators.leg3?.trajectory
    );
    next.simulators.planar2.presets = mergePlainObject(
      next.simulators.planar2.presets,
      saved.simulators.planar2?.presets
    );
    next.simulators.leg3.presets = mergePlainObject(
      next.simulators.leg3.presets,
      saved.simulators.leg3?.presets
    );
    return next;
  }

  // Backward compatibility for the previous single-mode 2 DOF app.
  const next = clone(defaultState);
  next.simulators.planar2.config = mergePlainObject(createDefaultPlanar2Config(), saved);
  return next;
}

export function loadSavedAppState() {
  const saved = safeParse(localStorage.getItem(STORAGE_KEYS.appState), null);
  return migrateLegacyState(saved);
}

export function saveAppState(appState) {
  localStorage.setItem(STORAGE_KEYS.appState, JSON.stringify(appState));
}

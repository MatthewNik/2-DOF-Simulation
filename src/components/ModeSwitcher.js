import { html } from "../lib.js";
import { SIMULATOR_MODES } from "../constants.js";

export function ModeSwitcher({ activeMode, onChange }) {
  return html`
    <div className="mode-switcher" role="tablist" aria-label="Simulator mode">
      ${SIMULATOR_MODES.map(
        (mode) => html`
          <button
            key=${mode.id}
            type="button"
            className=${activeMode === mode.id ? "active" : ""}
            onClick=${() => onChange(mode.id)}
          >
            ${mode.shortLabel}
          </button>
        `
      )}
    </div>
  `;
}

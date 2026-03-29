import { html } from "../lib.js";

export function ControlCard({
  id,
  title,
  subtitle,
  dragEnabled = false,
  dragging = false,
  dropTarget = false,
  onDragStart,
  onDragEnd,
  onDragOver,
  onDrop,
  children
}) {
  const className = [
    "control-card",
    dragEnabled ? "drag-enabled" : "drag-locked",
    dragging ? "dragging" : "",
    dropTarget ? "drop-target" : ""
  ]
    .filter(Boolean)
    .join(" ");

  return html`
    <section
      className=${className}
      onDragOver=${(event) => onDragOver?.(event, id)}
      onDrop=${(event) => onDrop?.(event, id)}
    >
      <header className="control-card-header">
        <div>
          <h3>${title}</h3>
          ${subtitle ? html`<p>${subtitle}</p>` : null}
        </div>
        <button type="button" className="drag-handle" aria-label=${`Drag ${title}`}>
          <span className="drag-handle-label">${dragEnabled ? "Move" : "Locked"}</span>
          <span
            className="drag-handle-grip"
            draggable=${dragEnabled}
            onDragStart=${dragEnabled ? (event) => onDragStart?.(event, id) : undefined}
            onDragEnd=${dragEnabled ? onDragEnd : undefined}
          >
          <span></span>
          <span></span>
          </span>
        </button>
      </header>
      <div className="control-card-body">${children}</div>
    </section>
  `;
}

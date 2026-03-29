import { html } from "../lib.js";

export function ControlCard({
  id,
  title,
  subtitle,
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
    dragging ? "dragging" : "",
    dropTarget ? "drop-target" : ""
  ]
    .filter(Boolean)
    .join(" ");

  return html`
    <section
      className=${className}
      draggable="true"
      onDragStart=${(event) => onDragStart?.(event, id)}
      onDragEnd=${onDragEnd}
      onDragOver=${(event) => onDragOver?.(event, id)}
      onDrop=${(event) => onDrop?.(event, id)}
    >
      <header className="control-card-header">
        <div>
          <h3>${title}</h3>
          ${subtitle ? html`<p>${subtitle}</p>` : null}
        </div>
        <button type="button" className="drag-handle" aria-label=${`Drag ${title}`}>
          <span></span>
          <span></span>
        </button>
      </header>
      <div className="control-card-body">${children}</div>
    </section>
  `;
}

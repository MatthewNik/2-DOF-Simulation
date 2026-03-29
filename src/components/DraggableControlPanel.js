import { html, useState } from "../lib.js";
import { ControlCard } from "./ControlCard.js";

export function DraggableControlPanel({ cards, order, onReorder }) {
  const [layoutEditEnabled, setLayoutEditEnabled] = useState(false);
  const [draggedId, setDraggedId] = useState(null);
  const [dropTargetId, setDropTargetId] = useState(null);
  const cardMap = new Map(cards.map((card) => [card.id, card]));

  function reorderCard(sourceId, targetId) {
    if (!sourceId || !targetId || sourceId === targetId) {
      return;
    }
    const currentOrder = order.filter((id) => cardMap.has(id));
    const fromIndex = currentOrder.indexOf(sourceId);
    const toIndex = currentOrder.indexOf(targetId);
    if (fromIndex === -1 || toIndex === -1) {
      return;
    }

    const nextOrder = [...currentOrder];
    nextOrder.splice(fromIndex, 1);
    nextOrder.splice(toIndex, 0, sourceId);
    onReorder(nextOrder);
  }

  const sortedCards = order
    .filter((id) => cardMap.has(id))
    .map((id) => cardMap.get(id))
    .concat(cards.filter((card) => !order.includes(card.id)));

  return html`
    <aside className="panel sidebar">
      <div className="sidebar-toolbar">
        <div>
          <h2>Control Modules</h2>
          <p>${layoutEditEnabled ? "Reorder mode enabled" : "Cards locked for editing controls"}</p>
        </div>
        <button
          type="button"
          className=${layoutEditEnabled ? "layout-toggle active" : "layout-toggle"}
          onClick=${() => {
            setLayoutEditEnabled((previous) => !previous);
            setDraggedId(null);
            setDropTargetId(null);
          }}
        >
          ${layoutEditEnabled ? "Lock Cards" : "Move Cards"}
        </button>
      </div>
      <div className="control-column">
        ${sortedCards.map(
          (card) => html`
            <${ControlCard}
              key=${card.id}
              id=${card.id}
              title=${card.title}
              subtitle=${card.subtitle}
              dragEnabled=${layoutEditEnabled}
              dragging=${draggedId === card.id}
              dropTarget=${dropTargetId === card.id && draggedId !== card.id}
              onDragStart=${(_event, id) => {
                if (!layoutEditEnabled) {
                  return;
                }
                setDraggedId(id);
                setDropTargetId(id);
              }}
              onDragEnd=${() => {
                setDraggedId(null);
                setDropTargetId(null);
              }}
              onDragOver=${(event, id) => {
                if (!layoutEditEnabled) {
                  return;
                }
                event.preventDefault();
                if (dropTargetId !== id) {
                  setDropTargetId(id);
                }
              }}
              onDrop=${(event, id) => {
                if (!layoutEditEnabled) {
                  return;
                }
                event.preventDefault();
                reorderCard(draggedId, id);
                setDraggedId(null);
                setDropTargetId(null);
              }}
            >
              ${card.content}
            </${ControlCard}>
          `
        )}
      </div>
    </aside>
  `;
}

import { html, useState } from "../lib.js";
import { ControlCard } from "./ControlCard.js";

export function DraggableControlPanel({ cards, order, onReorder }) {
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
      <div className="control-column">
        ${sortedCards.map(
          (card) => html`
            <${ControlCard}
              key=${card.id}
              id=${card.id}
              title=${card.title}
              subtitle=${card.subtitle}
              dragging=${draggedId === card.id}
              dropTarget=${dropTargetId === card.id && draggedId !== card.id}
              onDragStart=${(_event, id) => {
                setDraggedId(id);
                setDropTargetId(id);
              }}
              onDragEnd=${() => {
                setDraggedId(null);
                setDropTargetId(null);
              }}
              onDragOver=${(event, id) => {
                event.preventDefault();
                if (dropTargetId !== id) {
                  setDropTargetId(id);
                }
              }}
              onDrop=${(event, id) => {
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

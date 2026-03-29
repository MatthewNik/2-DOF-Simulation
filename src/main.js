import { createRoot } from "react-dom/client";
import "katex/dist/katex.min.css";
import "./styles.css";
import { html } from "./lib.js";
import { App } from "./components/App.js";

const root = createRoot(document.getElementById("root"));
root.render(html`<${App} />`);

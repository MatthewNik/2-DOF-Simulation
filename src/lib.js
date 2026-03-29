import React, {
  useDeferredValue,
  useEffect,
  useMemo,
  useRef,
  useState
} from "react";
import htm from "htm";
import katex from "katex";

const html = htm.bind(React.createElement);

function MathBlock({ tex }) {
  const markup = useMemo(
    () =>
      katex.renderToString(tex, {
        displayMode: true,
        throwOnError: false,
        strict: "ignore"
      }),
    [tex]
  );

  return html`<div className="math-block" dangerouslySetInnerHTML=${{ __html: markup }} />`;
}

export { html, MathBlock, useDeferredValue, useEffect, useMemo, useRef, useState };

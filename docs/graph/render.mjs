#!/usr/bin/env node
// render.mjs — genera el SVG del grafo a partir de graph.spec.json
//
//   npm install elkjs
//   node render.mjs graph.spec.json buey_graph.svg
//
// No hay una sola coordenada escrita a mano: ELK calcula posiciones de nodos,
// puertos y el ruteo ortogonal de cada cable. Para cambiar el diagrama se toca
// unicamente el .json.

import ELK from 'elkjs';
import { readFileSync, writeFileSync } from 'fs';

// ---------------------------------------------------------------- tema
const T = {
  bg: '#141416', card: '#1F1F23', cardBorder: '#32323A',
  pill: '#17171A', pillBorder: '#45454F',
  text: '#C6C6CF', name: '#EFEFF3', muted: '#7B7B87',
};
// el color sale del rol del nodo, no de la fila: un cable se pinta con el
// color del nodo que publica, asi se ve de que tipo es el dato que viaja.
// ------------------------------------------------------------- validar spec
// Se corre antes del layout para que un error de edicion salga como mensaje
// legible y no como un volcado de elkjs.
function validarSpec(spec) {
  const errs = [];
  const nodos = new Map(spec.nodes.map((n) => [n.id, n]));
  for (const n of spec.nodes) {
    if (!spec.layers[n.layer]) errs.push(`nodo "${n.id}": layer ${n.layer} no existe (hay ${spec.layers.length})`);
    if (!spec.roles[n.role]) errs.push(`nodo "${n.id}": role "${n.role}" no existe (validos: ${Object.keys(spec.roles).join(', ')})`);
    for (const lado of ['in', 'out']) {
      const dup = n[lado].filter((t, i) => n[lado].indexOf(t) !== i);
      if (dup.length) errs.push(`nodo "${n.id}": topico repetido en ${lado}: ${dup.join(', ')}`);
    }
  }
  spec.edges.forEach(([sn, st, tn, tt], i) => {
    const s2 = nodos.get(sn), t2 = nodos.get(tn);
    if (!s2) return errs.push(`cable ${i}: no existe el nodo origen "${sn}"`);
    if (!t2) return errs.push(`cable ${i}: no existe el nodo destino "${tn}"`);
    if (!s2.out.includes(st)) errs.push(`cable ${i}: "${sn}" no publica "${st}" (publica: ${s2.out.join(', ') || 'nada'})`);
    if (!t2.in.includes(tt)) errs.push(`cable ${i}: "${tn}" no escucha "${tt}" (escucha: ${t2.in.join(', ') || 'nada'})`);
    if (s2.layer === t2.layer) errs.push(`cable ${i}: "${sn}" y "${tn}" estan en la misma fila (${spec.layers[s2.layer]})`);
  });
  const usados = new Set(spec.edges.flatMap(([sn, st, tn, tt]) => [`${sn}|out|${st}`, `${tn}|in|${tt}`]));
  for (const n of spec.nodes) {
    for (const t of n.out) if (!usados.has(`${n.id}|out|${t}`)) errs.push(`aviso: "${n.id}" publica "${t}" y nadie lo escucha`);
  }
  const duros = errs.filter((e) => !e.startsWith('aviso'));
  errs.forEach((e) => console.error(duros.includes(e) ? `  ERROR: ${e}` : `  ${e}`));
  if (duros.length) {
    console.error(`\nFALLO: ${duros.length} error(es) en el spec. No se genero el SVG.`);
    process.exit(1);
  }
}
const roleColor = (id) => spec.roles[spec.nodes.find((n) => n.id === id).role].color;

// ------------------------------------------------------- metrica de texto
const W = { name: 6.9, pill: 6.15, label: 6.15 };
const textW = (s, k) => s.length * W[k];

const PAD = 12, PILL_H = 20, PILL_PAD = 9, GAP = 7, LABEL_W = 26, GAP_PILL = 6;

function measure(n) {
  const row = (arr) => arr.length
    ? LABEL_W + arr.reduce((a, t) => a + textW(t, 'pill') + 2 * PILL_PAD, 0) + GAP_PILL * (arr.length - 1)
    : 0;
  const inner = Math.max(row(n.in), row(n.out), textW(n.id, 'name'));
  const width = Math.round(inner + 2 * PAD);
  const height = PAD + PILL_H + GAP + 18 + (n.out.length ? GAP + PILL_H : 0) + PAD;
  return { width, height };
}

// posiciones de cada pill dentro del nodo
function pills(n, side) {
  const arr = n[side];
  let x = PAD + LABEL_W;
  const y = side === 'in' ? PAD : PAD + PILL_H + GAP + 18 + GAP;
  return arr.map((t) => {
    const w = textW(t, 'pill') + 2 * PILL_PAD;
    const p = { topic: t, x, y, w, h: PILL_H };
    x += w + GAP_PILL;
    return p;
  });
}

// ---------------------------------------------------------------- main
const spec = JSON.parse(readFileSync(process.argv[2] || 'graph.spec.json', 'utf8'));
validarSpec(spec);
const geom = new Map();
for (const n of spec.nodes) {
  const m = measure(n);
  geom.set(n.id, { ...m, in: pills(n, 'in'), out: pills(n, 'out'), node: n });
}

const elk = new ELK();
const graph = {
  id: 'root',
  layoutOptions: {
    'elk.algorithm': 'layered',
    'elk.direction': 'DOWN',
    'elk.edgeRouting': 'ORTHOGONAL',
    'elk.partitioning.activate': 'true',
    'elk.layered.spacing.nodeNodeBetweenLayers': '78',
    'elk.spacing.nodeNode': '44',
    'elk.spacing.edgeNode': '20',
    'elk.spacing.edgeEdge': '12',
    'elk.layered.spacing.edgeEdgeBetweenLayers': '12',
    'elk.layered.spacing.edgeNodeBetweenLayers': '20',
    'elk.layered.crossingMinimization.strategy': 'LAYER_SWEEP',
    'elk.layered.mergeEdges': process.env.MERGE || 'false',
    'elk.layered.crossingMinimization.semiInteractive': 'true',
    'elk.layered.nodePlacement.strategy': process.env.NP || 'BRANDES_KOEPF',
    ...(process.env.BK ? { 'elk.layered.nodePlacement.bk.fixedAlignment': process.env.BK } : {}),
    'elk.layered.considerModelOrder.strategy': 'NODES_AND_EDGES',
  },
  children: spec.nodes.map((n) => {
    const g = geom.get(n.id);
    const ports = [];
    for (const p of g.in) ports.push({ id: `${n.id}|in|${p.topic}`, x: p.x + p.w / 2, y: 0, width: 1, height: 1, layoutOptions: { 'elk.port.side': 'NORTH' } });
    for (const p of g.out) ports.push({ id: `${n.id}|out|${p.topic}`, x: p.x + p.w / 2, y: g.height, width: 1, height: 1, layoutOptions: { 'elk.port.side': 'SOUTH' } });
    return {
      id: n.id, width: g.width, height: g.height, ports,
      layoutOptions: { 'elk.portConstraints': 'FIXED_POS', 'elk.partitioning.partition': String(n.layer) },
    };
  }),
  edges: spec.edges.map((e, i) => ({
    id: `e${i}`, sources: [`${e[0]}|out|${e[1]}`], targets: [`${e[2]}|in|${e[3]}`],
    kind: e[4] || 'flow', srcNode: e[0],
  })),
};

const res = await elk.layout(graph);

// ------------------------------------------------------- sin recentrado
// Se probo centrar cada fila (ver render.mjs) y el costo es alto: un cable que
// iba recto entre dos filas desplazadas distinto tiene que ganar un escalon.
// Medido sobre este grafo: centrar pasa de 8 cables rectos a 0 y de 20 quiebres
// a 38. Aca se prioriza el trazado limpio y se acepta que las filas queden
// hasta ~120px descentradas entre si.
const CONTENT_W = res.width;

// ---------------------------------------------------------------- SVG
const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
// El SVG se ve en GitHub con la fuente del que mira. Declarar textLength hace
// que el navegador ajuste el texto al ancho que asumio el layout, asi nunca
// desborda su pill aunque la fuente no sea la que se uso para medir.
const fit = (t, w) => Math.round(t.length * w * 10) / 10;
const M = 34;
const GUTTER = 90;                      // canal para las etiquetas de capa
const W_SVG = Math.ceil(res.width) + 2 * (M + GUTTER);
const H_SVG = Math.ceil(res.height) + 2 * M;
const OX = M + GUTTER, OY = M;

const out = [];
out.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W_SVG}" height="${H_SVG}" viewBox="0 0 ${W_SVG} ${H_SVG}" font-family="system-ui,-apple-system,Segoe UI,sans-serif">`);
out.push(`<rect width="${W_SVG}" height="${H_SVG}" fill="${T.bg}"/>`);

// cables (van primero, quedan por debajo de las cajas)
const R = 7;
function roundedPath(pts) {
  if (pts.length < 3) return `M${pts.map((p) => `${p.x},${p.y}`).join(' L')}`;
  let d = `M${pts[0].x},${pts[0].y}`;
  for (let i = 1; i < pts.length - 1; i++) {
    const a = pts[i - 1], b = pts[i], c = pts[i + 1];
    const l1 = Math.hypot(b.x - a.x, b.y - a.y), l2 = Math.hypot(c.x - b.x, c.y - b.y);
    const r = Math.min(R, l1 / 2, l2 / 2);
    const p1 = { x: b.x - (b.x - a.x) / l1 * r, y: b.y - (b.y - a.y) / l1 * r };
    const p2 = { x: b.x + (c.x - b.x) / l2 * r, y: b.y + (c.y - b.y) / l2 * r };
    d += ` L${p1.x.toFixed(1)},${p1.y.toFixed(1)} Q${b.x},${b.y} ${p2.x.toFixed(1)},${p2.y.toFixed(1)}`;
  }
  const e = pts[pts.length - 1];
  return d + ` L${e.x},${e.y}`;
}

const nodeById = new Map(res.children.map((c) => [c.id, c]));

// canal exterior para los lazos que van contra el flujo
const RING_X = Math.max(...res.children.map((c) => c.x + c.width)) + 34;
const RING_Y = Math.min(...res.children.map((c) => c.y)) - 26;
const portXY = (id) => {
  const [nid, side, topic] = id.split('|');
  const c = nodeById.get(nid), g = geom.get(nid);
  const p = g[side].find((q) => q.topic === topic);
  return { x: c.x + p.x + p.w / 2 + OX, y: c.y + (side === 'in' ? 0 : g.height) + OY };
};

for (const e of res.edges) {
  const spec_e = graph.edges.find((g) => g.id === e.id);
  const color = roleColor(spec_e.srcNode);
  const dash = spec_e.kind === 'feedback' ? ' stroke-dasharray="5 4"' : '';
  const tag0 = `${spec_e.srcNode}:${spec_e.sources[0].split('|').pop()} -> ${spec_e.targets[0].split('|')[0]}`;

  // Un lazo cruza varias bandas de golpe; como cada banda se centra por
  // separado, no puede quedar recto. Se lo saca por el anillo exterior.
  if (spec_e.kind === 'feedback') {
    const s = portXY(spec_e.sources[0]), t = portXY(spec_e.targets[0]);
    const pts = [s, { x: s.x, y: s.y + 26 }, { x: RING_X + OX, y: s.y + 26 },
                 { x: RING_X + OX, y: RING_Y + OY }, { x: t.x, y: RING_Y + OY }, t];
    out.push(`<path data-edge="${esc(tag0)}" d="${roundedPath(pts)}" fill="none" stroke="${color}" stroke-width="1.6" stroke-linecap="round"${dash}/>`);
    out.push(`<path d="M${t.x - 4},${t.y - 6} L${t.x},${t.y} L${t.x + 4},${t.y - 6}" fill="none" stroke="${color}" stroke-width="1.6" stroke-linejoin="round" stroke-linecap="round"/>`);
    continue;
  }

  for (const s of e.sections || []) {
    const pts = [s.startPoint, ...(s.bendPoints || []), s.endPoint]
      .map((p) => ({ x: Math.round((p.x + OX) * 10) / 10, y: Math.round((p.y + OY) * 10) / 10 }));
    const tag = `${spec_e.srcNode}:${spec_e.sources[0].split('|').pop()} -> ${spec_e.targets[0].split('|')[0]}`;
    out.push(`<path data-edge="${esc(tag)}" d="${roundedPath(pts)}" fill="none" stroke="${color}" stroke-width="1.6" stroke-linecap="round"${dash}/>`);
    const end = pts[pts.length - 1], prev = pts[pts.length - 2];
    const dy = Math.sign(end.y - prev.y) || 1;   // los puertos son N/S: la llegada es vertical
    out.push(`<path d="M${end.x - 4},${end.y - dy * 6} L${end.x},${end.y} L${end.x + 4},${end.y - dy * 6}" fill="none" stroke="${color}" stroke-width="1.6" stroke-linejoin="round" stroke-linecap="round"/>`);
  }
}

// cajas
for (const c of res.children) {
  const g = geom.get(c.id), n = g.node;
  const X = c.x + OX, Y = c.y + OY;
  const accent = roleColor(n.id);
  out.push(`<g>`);
  out.push(`<rect x="${X}" y="${Y}" width="${g.width}" height="${g.height}" fill="${T.card}" stroke="${T.cardBorder}" stroke-width="1"/>`);
  out.push(`<rect x="${X}" y="${Y}" width="3" height="${g.height}" fill="${accent}"/>`);
  const drawRow = (arr, label) => {
    if (!arr.length) return;
    const yc = Y + arr[0].y + PILL_H / 2;
    out.push(`<text x="${X + PAD + LABEL_W - 8}" y="${yc}" fill="${T.muted}" font-size="11" text-anchor="end" dominant-baseline="central">${label}</text>`);
    for (const p of arr) {
      out.push(`<rect x="${X + p.x}" y="${Y + p.y}" width="${p.w}" height="${p.h}" fill="${T.pill}" stroke="${T.pillBorder}" stroke-width="0.8"/>`);
      out.push(`<text x="${X + p.x + p.w / 2}" y="${yc}" fill="${T.text}" font-size="11" text-anchor="middle" dominant-baseline="central" textLength="${fit(p.topic, W.pill)}" lengthAdjust="spacingAndGlyphs">${esc(p.topic)}</text>`);
    }
  };
  drawRow(g.in, 'in');
  drawRow(g.out, 'out');
  out.push(`<text x="${X + g.width / 2}" y="${Y + PAD + PILL_H + GAP + 9}" fill="${T.name}" font-size="12.5" font-weight="500" text-anchor="middle" dominant-baseline="central" textLength="${fit(n.id, W.name)}" lengthAdjust="spacingAndGlyphs">${esc(n.id)}</text>`);
  out.push(`</g>`);
}

// etiquetas de capa
const byLayer = new Map();
for (const c of res.children) {
  const l = geom.get(c.id).node.layer;
  byLayer.set(l, Math.min(byLayer.get(l) ?? Infinity, c.y));
}
for (const [l, y] of byLayer) {
  out.push(`<text x="${OX - 24}" y="${y + OY + 26}" fill="${T.muted}" font-size="11.5" text-anchor="end">${spec.layers[l]}</text>`);
}

// ------------------------------------------------------------- validacion
// Chequeo duro: cada cable tiene que empezar y terminar exactamente en el
// centro del socket que dice conectar. Si esto falla, el diagrama miente.
let errs = 0;
for (const e of res.edges) {
  const g = graph.edges.find((q) => q.id === e.id);
  if (g.kind === 'feedback') continue;
  const s0 = portXY(g.sources[0]), t0 = portXY(g.targets[0]);
  for (const sec of e.sections || []) {
    const a = { x: sec.startPoint.x + OX, y: sec.startPoint.y + OY };
    const b = { x: sec.endPoint.x + OX, y: sec.endPoint.y + OY };
    if (Math.abs(a.x - s0.x) > 1 || Math.abs(b.x - t0.x) > 1) {
      console.error(`  DESALINEADO  ${g.sources[0]} -> ${g.targets[0]}  (dx ini ${(a.x - s0.x).toFixed(1)}, dx fin ${(b.x - t0.x).toFixed(1)})`);
      errs++;
    }
  }
}
if (errs) {
  console.error(`FALLO: ${errs} cables no terminan en su socket. El SVG no se escribio.`);
  process.exit(1);
}
console.error('  ok: todos los cables alineados con su socket');

out.push('</svg>');
writeFileSync(process.argv[3] || 'buey_graph.svg', out.join('\n'));
console.error(`ok  ${W_SVG}x${H_SVG}  ${res.children.length} nodos  ${res.edges.length} cables`);

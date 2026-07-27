#!/usr/bin/env node
// sync-docs.mjs — vuelca el contenido generado dentro de ../ARCHITECTURE.md
//
//   node sync-docs.mjs
//
// Reemplaza lo que haya entre los marcadores <!-- graph:start --> y
// <!-- graph:end -->. El resto del documento no se toca, asi que se puede
// escribir prosa alrededor sin miedo a perderla en el proximo build.

import { readFileSync, writeFileSync, existsSync } from 'fs';

const SPEC = 'graph.spec.json';
const DOC = '../ARCHITECTURE.md';
const IMG = 'graph/buey_graph.svg';   // relativo a docs/
const START = '<!-- graph:start -->';
const END = '<!-- graph:end -->';

if (!existsSync(DOC)) {
  console.error(`FALLO: no existe ${DOC}. Crealo con los marcadores:\n\n${START}\n${END}\n`);
  process.exit(1);
}

const spec = JSON.parse(readFileSync(SPEC, 'utf8'));
const doc = readFileSync(DOC, 'utf8');
const i = doc.indexOf(START), j = doc.indexOf(END);
if (i < 0 || j < 0 || j < i) {
  console.error(`FALLO: faltan los marcadores en ${DOC}. Agregalos donde va el contenido generado:\n\n${START}\n${END}\n`);
  process.exit(1);
}

// ---------------------------------------------------------------- contenido
const pill = (t) => '`' + t + '`';
const L = [];

L.push('');
L.push('> Generado por `docs/graph/`. No editar a mano: se pisa en el proximo');
L.push('> `npm run build`. Para cambiar el diagrama se edita `docs/graph/graph.spec.json`.');
L.push('');
L.push(`![Arquitectura de buey_robot](${IMG})`);
L.push('');
L.push('## Nodos');
L.push('');
L.push('| Nodo | Capa | Archivo | Entra | Sale | Que hace |');
L.push('|---|---|---|---|---|---|');
for (const n of spec.nodes) {
  L.push(`| **${n.id}** | ${spec.layers[n.layer]} | \`${n.file}\` | ${n.in.map(pill).join(' ') || '—'} | ${n.out.map(pill).join(' ') || '—'} | ${n.desc} |`);
}

L.push('');
L.push('## Topicos');
L.push('');
L.push('| Topico | Publica | Escuchan |');
L.push('|---|---|---|');
const topics = new Map();
for (const [sn, st, tn] of spec.edges) {
  const k = st;
  const o = topics.get(k) || { pub: new Set(), sub: new Set() };
  o.pub.add(sn); o.sub.add(tn);
  topics.set(k, o);
}
for (const [t, o] of [...topics].sort((a, b) => a[0].localeCompare(b[0]))) {
  L.push(`| \`${t}\` | ${[...o.pub].join(', ')} | ${[...o.sub].join(', ')} |`);
}

L.push('');
L.push('## Colores');
L.push('');
L.push('El color de un cable es el del nodo que publica.');
L.push('');
L.push('| Rol | Significado |');
L.push('|---|---|');
for (const [r, v] of Object.entries(spec.roles)) L.push(`| ${r} | ${v.desc} |`);
L.push('');

const out = doc.slice(0, i + START.length) + '\n' + L.join('\n') + '\n' + doc.slice(j);
if (out === doc) {
  console.error('  ARCHITECTURE.md ya estaba al dia');
} else {
  writeFileSync(DOC, out);
  console.error(`  ARCHITECTURE.md actualizado (${spec.nodes.length} nodos, ${topics.size} topicos)`);
}

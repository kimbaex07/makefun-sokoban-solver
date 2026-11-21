// Sokoban solver — make.fun sokoban-solver-01-run-20261121.
//
// Strategy. The generator builds every level as one hallway row plus 1-4
// independent rooms (3-4 boxes each). A box pushed out of its room can never
// be pushed back, so the level's push-optimal count P* is the sum of the
// rooms' optima. We therefore:
//   1. split the level into rooms (connected floor components off the
//      hallway row, each entered through one door),
//   2. solve each room push-optimally with A* over pushes (min-assignment
//      heuristic over BFS box-to-goal distances, corner + 2x2-freeze deadlock
//      pruning), requiring the player to still reach the door afterwards
//      (except for the last room),
//   3. stitch the rooms' push sequences together with box-free walks
//      through the hallway, expanding every push into walk moves + the push.
//
// Pure Node builtins, deterministic: every budget is a count of expanded
// states, never the wall clock, so two runs write identical bytes.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];

// Work budgets (expanded states). Deterministic by construction.
const OPT_CAP = 150000;
const WEIGHTED_CAP = 150000;
const GREEDY_CAP = 300000;
const GLOBAL_CAP = 3000000;

let expansionsTotal = 0;

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goals = new Set();
  const boxes = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.add(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// BFS distances from `start` over cells where passable[i] is 1.
function bfsDist(start, passable, w, h) {
  const dist = new Int32Array(w * h).fill(-1);
  const q = [start];
  dist[start] = 0;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    const x = c % w;
    const y = (c / w) | 0;
    if (x > 0 && passable[c - 1] && dist[c - 1] < 0) { dist[c - 1] = dist[c] + 1; q.push(c - 1); }
    if (x + 1 < w && passable[c + 1] && dist[c + 1] < 0) { dist[c + 1] = dist[c] + 1; q.push(c + 1); }
    if (y > 0 && passable[c - w] && dist[c - w] < 0) { dist[c - w] = dist[c] + 1; q.push(c - w); }
    if (y + 1 < h && passable[c + w] && dist[c + w] < 0) { dist[c + w] = dist[c] + 1; q.push(c + w); }
  }
  return dist;
}

// Flood fill from `start` over passable cells, treating blocked[i] as walls.
// Returns { reach: Uint8Array, rep: smallest reachable cell index }.
function flood(start, passable, blocked, w, h) {
  const reach = new Uint8Array(w * h);
  const q = [start];
  reach[start] = 1;
  let rep = start;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    if (c < rep) rep = c;
    const x = c % w;
    const y = (c / w) | 0;
    const step = (n) => {
      if (passable[n] && !blocked[n] && !reach[n]) { reach[n] = 1; q.push(n); }
    };
    if (x > 0) step(c - 1);
    if (x + 1 < w) step(c + 1);
    if (y > 0) step(c - w);
    if (y + 1 < h) step(c + w);
  }
  return { reach, rep };
}

// Shortest walk (u/d/l/r) from `from` to `to` over passable cells, never
// stepping onto a blocked cell. Returns the move string, or null.
function walkPath(from, to, passable, blocked, w, h) {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-1);
  const prevMove = new Array(w * h).fill('');
  const q = [from];
  prev[from] = from;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    if (c === to) break;
    const x = c % w;
    const y = (c / w) | 0;
    for (const [dx, dy, letter] of DIRS) {
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const n = ny * w + nx;
      if (!passable[n] || blocked[n] || prev[n] >= 0) continue;
      prev[n] = c;
      prevMove[n] = letter;
      q.push(n);
    }
  }
  if (prev[to] < 0) return null;
  let moves = '';
  for (let c = to; c !== from; c = prev[c]) moves = prevMove[c] + moves;
  return moves;
}

// Split a level into rooms: connected floor components off the hallway row
// (the row the player starts on). Returns null when the shape is unexpected.
function decompose(level) {
  const { w, h, wall, goals, boxes } = level;
  const hr = (level.player / w) | 0;
  const seen = new Int32Array(w * h).fill(-1);
  const rooms = [];
  let compCount = 0;
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || ((i / w) | 0) === hr || seen[i] >= 0) continue;
    const cid = compCount++;
    const cells = [];
    const q = [i];
    seen[i] = cid;
    for (let head = 0; head < q.length; head++) {
      const c = q[head];
      cells.push(c);
      const x = c % w;
      const y = (c / w) | 0;
      const step = (n) => {
        if (!wall[n] && ((n / w) | 0) !== hr && seen[n] < 0) { seen[n] = cid; q.push(n); }
      };
      if (x > 0) step(c - 1);
      if (x + 1 < w) step(c + 1);
      if (y > 0) step(c - w);
      if (y + 1 < h) step(c + w);
    }
    const roomBoxes = boxes.filter((b) => seen[b] === cid);
    const roomGoals = [...goals].filter((g) => seen[g] === cid);
    if (roomBoxes.length === 0 && roomGoals.length === 0) continue;
    if (roomBoxes.length === 0 || roomBoxes.length !== roomGoals.length) return null;
    // Door cells: room cells adjacent to a hallway-row floor cell.
    const doors = [];
    const entries = [];
    for (const c of cells) {
      for (const n of [c - w, c + w]) {
        if (n < 0 || n >= w * h) continue;
        if (((n / w) | 0) === hr && !wall[n]) {
          if (!doors.includes(c)) doors.push(c);
          if (!entries.includes(n)) entries.push(n);
        }
      }
    }
    if (doors.length === 0) return null;
    rooms.push({ cells, boxes: roomBoxes, goals: roomGoals, doors, entries });
  }
  // Every box and goal must belong to exactly one room.
  if (boxes.some((b) => seen[b] < 0)) return null;
  if ([...goals].some((g) => seen[g] < 0)) return null;
  rooms.sort((a, b) => Math.min(...a.doors) - Math.min(...b.doors));
  return rooms;
}

// A* over pushes for one room. `startPlayer` is the entry cell the player
// stands on. When needExit is true the goal also requires the player to be
// able to reach an entry cell afterwards. `weight` scales the heuristic
// (1 = optimal, larger = greedier). Returns the push list or null.
function solveRoom(level, room, startPlayer, needExit, cap, weight) {
  const { w, h } = level;
  const roomMask = new Uint8Array(w * h);
  for (const c of room.cells) roomMask[c] = 1;
  const floorMask = new Uint8Array(w * h);
  for (const c of room.cells) floorMask[c] = 1;
  for (const c of room.entries) floorMask[c] = 1;
  const goalSet = new Set(room.goals);
  const n = room.boxes.length;

  // Static dead cells: non-goal room cells in a corner (two perpendicular
  // sides blocked for a box; entry cells count as blocked for boxes).
  const blockedForBox = (c) => c < 0 || c >= w * h || !roomMask[c];
  const dead = new Uint8Array(w * h);
  for (const c of room.cells) {
    if (goalSet.has(c)) continue;
    const x = c % w;
    const y = (c / w) | 0;
    const up = y === 0 || blockedForBox(c - w);
    const down = y + 1 >= h || blockedForBox(c + w);
    const left = x === 0 || blockedForBox(c - 1);
    const right = x + 1 >= w || blockedForBox(c + 1);
    if ((up && left) || (up && right) || (down && left) || (down && right)) dead[c] = 1;
  }

  // BFS box-to-goal distances over the room floor (ignoring other boxes).
  const dists = room.goals.map((g) => bfsDist(g, floorMask, w, h));
  // Permutations of goal indices for exact min-assignment (n <= 5).
  const perms = [];
  if (n <= 5) {
    const idx = [...Array(n).keys()];
    const gen = (arr, k) => {
      if (k === 1) { perms.push([...arr]); return; }
      for (let i = 0; i < k; i++) {
        gen(arr, k - 1);
        const j = k % 2 === 0 ? i : 0;
        [arr[j], arr[k - 1]] = [arr[k - 1], arr[j]];
      }
    };
    gen(idx, n);
  }
  const heuristic = (boxes) => {
    let best = Infinity;
    if (n <= 5) {
      for (const p of perms) {
        let s = 0;
        for (let i = 0; i < n; i++) {
          const d = dists[p[i]][boxes[i]];
          if (d < 0) { s = Infinity; break; }
          s += d;
        }
        if (s < best) best = s;
      }
    } else {
      let s = 0;
      for (let i = 0; i < n; i++) {
        let m = Infinity;
        for (let g = 0; g < n; g++) {
          const d = dists[g][boxes[i]];
          if (d >= 0 && d < m) m = d;
        }
        if (m === Infinity) return Infinity;
        s += m;
      }
      best = s;
    }
    return best;
  };

  // 2x2 freeze: a 2x2 block of walls/boxes holding a box off its goal.
  const frozen = (boxes, boxSet) => {
    for (const b of boxes) {
      const x = b % w;
      const y = (b / w) | 0;
      for (const [ox, oy] of [[-1, -1], [0, -1], [-1, 0], [0, 0]]) {
        const tx = x + ox;
        const ty = y + oy;
        if (tx < 0 || tx + 1 >= w || ty < 0 || ty + 1 >= h) continue;
        const tl = ty * w + tx;
        const sq = [tl, tl + 1, tl + w, tl + w + 1];
        let allBlocked = true;
        let badBox = false;
        for (const c of sq) {
          if (boxSet.has(c)) {
            if (!goalSet.has(c)) badBox = true;
          } else if (roomMask[c]) {
            allBlocked = false;
          }
        }
        if (allBlocked && badBox) return true;
      }
    }
    return false;
  };

  const startBoxes = [...room.boxes].sort((a, b) => a - b);
  const startBlocked = new Uint8Array(w * h);
  for (const b of startBoxes) startBlocked[b] = 1;
  const startReach = flood(startPlayer, floorMask, startBlocked, w, h);
  const keyOf = (boxes, rep) => boxes.join(',') + '#' + rep;

  // State storage.
  const S_boxes = [startBoxes];
  const S_player = [startPlayer];
  const S_g = [0];
  const S_parent = [-1];
  const S_pushBox = [-1];
  const S_pushDir = [-1];
  const best = new Map([[keyOf(startBoxes, startReach.rep), 0]]);

  // Binary heap on [f, seq, stateIdx].
  const heap = [];
  let seq = 0;
  const heapLess = (a, b) => a[0] < b[0] || (a[0] === b[0] && a[1] < b[1]);
  const heapPush = (e) => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i], heap[p])) break;
      [heap[i], heap[p]] = [heap[p], heap[i]];
      i = p;
    }
  };
  const heapPop = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1], heap[c])) c++;
        if (!heapLess(heap[c], heap[i])) break;
        [heap[i], heap[c]] = [heap[c], heap[i]];
        i = c;
      }
    }
    return top;
  };

  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  heapPush([weight * h0, seq++, 0]);

  let expanded = 0;
  while (heap.length > 0) {
    const [, , si] = heapPop();
    const boxes = S_boxes[si];
    const player = S_player[si];
    const g = S_g[si];
    const blocked = new Uint8Array(w * h);
    for (const b of boxes) blocked[b] = 1;
    const { reach, rep } = flood(player, floorMask, blocked, w, h);
    if (best.get(keyOf(boxes, rep)) !== g) continue;

    // Goal test.
    if (boxes.every((b) => goalSet.has(b))) {
      if (!needExit) return finish(si);
      let canExit = false;
      for (const e of room.entries) if (reach[e]) { canExit = true; break; }
      if (canExit) return finish(si);
    }

    if (expanded >= cap || expansionsTotal >= GLOBAL_CAP) return null;
    expanded++;
    expansionsTotal++;

    const boxSet = new Set(boxes);
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const [dx, dy] = DIRS[d];
        const nx = bx + dx;
        const ny = by + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const dest = ny * w + nx;
        if (!roomMask[dest] || boxSet.has(dest)) continue;
        const behind = b - (dy * w + dx);
        if (!reach[behind]) continue;
        const nb = [...boxes];
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        if (nb.some((c) => dead[c])) continue;
        const nbSet = new Set(nb);
        if (frozen(nb, nbSet)) continue;
        const hh = heuristic(nb);
        if (hh === Infinity) continue;
        const nBlocked = new Uint8Array(w * h);
        for (const c of nb) nBlocked[c] = 1;
        const nRep = flood(b, floorMask, nBlocked, w, h).rep;
        const k = keyOf(nb, nRep);
        const seenG = best.get(k);
        if (seenG !== undefined && seenG <= g + 1) continue;
        best.set(k, g + 1);
        const ni = S_boxes.length;
        S_boxes.push(nb);
        S_player.push(b);
        S_g.push(g + 1);
        S_parent.push(si);
        S_pushBox.push(b);
        S_pushDir.push(d);
        heapPush([g + 1 + weight * hh, seq++, ni]);
      }
    }
  }
  return null;

  function finish(si) {
    const pushes = [];
    for (let s = si; s > 0; s = S_parent[s]) {
      pushes.push({ box: S_pushBox[s], dir: S_pushDir[s] });
    }
    pushes.reverse();
    return { pushes };
  }
}

function solveLevel(text) {
  const level = parseLevel(text);
  if (level.boxes.length === 0) return '';
  const rooms = decompose(level);
  if (!rooms) return '';
  const { w, h, wall } = level;
  const floorAll = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) floorAll[i] = wall[i] ? 0 : 1;
  const boxesNow = new Set(level.boxes);
  const boxMask = () => {
    const m = new Uint8Array(w * h);
    for (const b of boxesNow) m[b] = 1;
    return m;
  };
  let cur = level.player;
  const moves = [];
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri];
    const needExit = ri < rooms.length - 1;
    const entry = Math.min(...room.entries);
    const roomFloor = new Uint8Array(w * h);
    for (const c of room.cells) roomFloor[c] = 1;
    for (const c of room.entries) roomFloor[c] = 1;

    const toEntry = walkPath(cur, entry, floorAll, boxMask(), w, h);
    if (toEntry === null) return '';
    moves.push(toEntry);
    cur = entry;

    let sol = solveRoom(level, room, cur, needExit, OPT_CAP, 1);
    if (!sol) sol = solveRoom(level, room, cur, needExit, WEIGHTED_CAP, 2.5);
    if (!sol) sol = solveRoom(level, room, cur, needExit, GREEDY_CAP, 100);
    if (!sol) return '';

    for (const push of sol.pushes) {
      const [dx, dy, letter] = DIRS[push.dir];
      const behind = push.box - (dy * w + dx);
      const dest = push.box + (dy * w + dx);
      const walk = walkPath(cur, behind, roomFloor, boxMask(), w, h);
      if (walk === null) return '';
      moves.push(walk);
      moves.push(letter);
      boxesNow.delete(push.box);
      boxesNow.add(dest);
      cur = push.box;
    }
  }
  return moves.join('');
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}

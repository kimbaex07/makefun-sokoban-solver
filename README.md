# Sokoban solver — make.fun sokoban-solver-01-run-20261121

One program (`main.ts`, Node 22, no dependencies) that answers every
instance of a set in a single run: it reads `/input/<id>/level.txt` and
writes `/output/<id>/solution.txt` with the player's moves (u/d/l/r).

## Idea

The generator builds every level as one hallway row plus 1–4 independent
rooms of 3–4 boxes each. A box pushed out of its room can never be pushed
back, so a level's push-optimal count is the sum of its rooms' optima.
The solver therefore:

1. splits the level into rooms (connected floor components off the hallway
   row, each entered through one door);
2. solves each room push-optimally with A* over pushes — a min-assignment
   heuristic over BFS box-to-goal distances, with corner and 2×2-freeze
   deadlock pruning — and requires the player to still reach the door
   afterwards (except for the last room);
3. stitches the rooms' push sequences together with box-free walks through
   the hallway, expanding every push into walk moves plus the push itself.

If optimal A* exhausts its work budget on a room, the solver falls back to
weighted A* and then to greedy best-first, so a room still yields a
(possibly longer) solution instead of nothing. All budgets are counts of
expanded states, never the wall clock, so two runs write identical bytes.

## Local test

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

or with the kit:

```sh
node cli/makefun.mjs grade-local --kit <kit> --bot <this repo> --driver process
```

## License

MIT — see LICENSE.

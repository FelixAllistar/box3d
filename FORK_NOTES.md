# Fork notes (FelixAllistar/box3d)

Our changes on top of upstream `erincatto/box3d`. Last rebased onto upstream
`3362633` (Dynamic Mover, #202) on 2026-11-05.

| Commit | What | Upstream status |
| --- | --- | --- |
| Add Box3D web demo and benchmark outputs | demo/bench files | ours only |
| Raise Box3D world capacity for vectorized training | bigger world limits for many envs | ours only |
| Preserve original edge behavior in independently owned mesh subsets | mesh subset edge flags + `test_mesh_subset.c` | ours only |
| Fail hull construction instead of spinning on degenerate face merges | hull-builder hang fix (below) | **not reported yet** |

## Hull-builder hang (`2cf5b4a`)

**TL;DR:** `b3CreateHull` could freeze forever in release builds. Now it
returns NULL instead.

- When building a convex hull, Box3D merges neighbouring faces. To merge two
  faces it walks along the edges they share
  (`b3HullBuilder_ConnectFaces`, the two `while (...->twin->face == twin->face)`
  loops).
- If the two faces share **all** their edges (a nearly flat sliver input), the
  walk goes round forever. Debug builds have asserts that catch it; release
  builds compile them out, so the program just hangs.
- Fix: if the walk comes back to the starting edge, mark the builder failed,
  stop merging, and return NULL. Callers already handle a NULL hull.
- General issue: any caller of `b3CreateHull` with an unlucky input can hit it.
- Found by SWAT (pufferlib `ocean/swat`): cutting a bullet hole into a door
  leaf that was swung almost fully open, then rebuilding its fracture pieces.
- Caveat: the exact input points were not saved, so it is not proven the input
  was "reasonable" rather than a bad sliver from our cutter. Either way, a
  library hang is worse than a refusal.

Possible note for Erin, if/when Felix decides to mention it (do not open a PR
without asking Felix):

> I hit an infinite loop in `b3CreateHull` (release build). In
> `b3HullBuilder_ConnectFaces`, the `edgePrev`/`edgeNext` walks never end when
> the two faces share their whole boundary (near-flat sliver inputs). The
> asserts catch it in debug but are compiled out in release. Local fix: detect
> the wrap back to the start edge, flag failure, return NULL from
> `b3CreateHull`. Found while fracturing thin door panels in a game.

To capture a repro: log the `points` passed to `b3CreateHull` whenever it
returns NULL (SWAT: `swat_sections_hulls` in `ocean/swat/world.c`).

## Updating from upstream

```bash
git fetch upstream
git switch -c upstream-YYYY-MM-DD
git rebase upstream/main          # our commits on top; resolve, keep both sides of test lists
cmake -S . -B build-next -DCMAKE_BUILD_TYPE=Release -DBOX3D_UNIT_TESTS=ON
cmake --build build-next -j8 && build-next/bin/test
```

Then run SWAT's door suites against `build-next/src/libbox3d.a` before
rebuilding `build/` (SWAT links `build/src/libbox3d.a` and reads headers from
this source tree).

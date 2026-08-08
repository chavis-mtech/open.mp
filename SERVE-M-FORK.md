# SERVE-M fork of open.mp

This is `chavis-mtech/open.mp`, branch `serve-m`. It exists so SERVE-M can ship NPC
behaviour that upstream does not have yet, **without falling behind upstream**. Staying
mergeable matters more than any individual change here: a fork that drifts stops receiving
fixes, and the point of this one is to keep receiving them.

`upstream` = `openmultiplayer/open.mp`. Never push there.

## The whole delta

Eight files. Everything else is byte-identical to upstream, and it should stay that way —
if a change can be made in SERVE-M's own C++ instead of here, make it there.

```
CMakeLists.txt                        +3    clang-cl /EHsc
Server/Source/CMakeLists.txt          +3    macOS: dl without libatomic
Server/Components/NPCs/NPC/npc.cpp    heavy the driving, sync and damage work below
Server/Components/NPCs/NPC/npc.hpp    +10
Server/Components/NPCs/Node/node.cpp  ~168  directional / lane-aware node traversal
Server/Components/NPCs/Node/node.hpp  +15
Server/Components/NPCs/npcs_impl.cpp  +6    damage animation wiring
Server/Components/NPCs/utils.hpp      +27   weapon damage normalisation
```

## A. Upstream bugs fixed here

These are not SERVE-M behaviour — they are defects in upstream that SERVE-M happened to hit
first. **Each one that gets upstreamed permanently shrinks this fork**, so they are the
first thing to revisit whenever there is time for a PR.

| What | Upstream today |
|---|---|
| `NPC::getVelocity()` | returns `player_->getPosition()` — callers read world coordinates as a velocity. Predictive avoidance projected NPCs kilometres away and could never see a converging pair. |
| `NPC::shoot()` | passes the shooter's current `weapon` to the damage event but `bulletData.weapon` to `processDamage`, so the event and the damage disagree about which weapon fired. |
| `NPC::move()` | declares a local `float moveSpeed_`, shadowing the member. The member keeps its previous value and the requested speed is silently dropped. |
| `NPC::sendFootSync()` | keys off the vehicle pointer alone. A foot packet emitted after `putInVehicle` beats the driver packet on remote clients: the ped stands at the vehicle origin with their legs through the floor. |
| `NPC::sendDriverSync()` | sends `velocity_` raw. Internal velocity is per-millisecond; the packet field is per-20ms-frame. Observers extrapolate a near-stationary car that each packet snaps forward — in-game, stutter and floating vehicles. |
| macOS link | links `atomic` alongside `dl`; libc++/compiler-rt provide atomics and no separate libatomic exists there. |

## B. SERVE-M behaviour

Expected to stay in the fork.

- **Bounded-rate steering for driven NPCs** — `advance()`, `move()`, `stopMove()`. Upstream
  replaces the vehicle quaternion with the next node's heading, so every graph edge looks
  like the car was grabbed and rotated in place. Here the heading slews at a bounded rate,
  speed drops for tight corners, and arrival additionally requires facing roughly the right
  way, which produces an arc between road links.
- **Directional, lane-aware node traversal** — `node.cpp`, `node.hpp`, `changeNode()`,
  `updateNodePoint()`, `playNode()`. Follows link direction, treats point zero as valid,
  exhausts directional links, preserves paused destinations, and spreads drivers across the
  lanes a road actually has.
- **Seat-aware sync** — `putInVehicle()`, `removeFromVehicle()`, `enterVehicle()`,
  `exitVehicle()`, `sendPassengerSync()`. Drivers no longer briefly render on foot inside
  their own vehicle.
- **Damage animations** — `processDamage()`, `kill()`, `getAnimation()`, `utils.hpp`,
  `npcs_impl.cpp`. NPCs react to being hit instead of absorbing bullets impassively.

## C. Considered and deliberately not taken

From the earlier `serve-m` line (`8860acca`), reconciled in `5cb485cd`. Recorded because
they are genuinely good and should be reconsidered once there is a way to check them:

- **Pursuit steering** — chases a lookahead point on the current segment so cross-track
  error decays, bounds yaw by speed and turn radius rather than a flat rate, and adds
  `npc.drive_z_clearance` / `npc.max_lane_offset`. A better model than what is here. It is
  a rewrite of the same function, and nothing distinguishes it from the current one except
  watching NPC traffic on a real client.
- **Removing the teleport at the start of `playNode()`** — the snap is a visible warp (a
  punched ped blinking sideways before fleeing). SERVE-M's `NpcDirector` currently leans on
  that teleport in three places: it picks node starts 12–22 m away and compensates for the
  resulting "first bounce". Removing it here alone changes NPC behaviour in ways only
  in-game testing can judge; the two repositories have to change together.

## D. Updating from upstream

```bash
cd third_party/open.mp-server
git fetch upstream
git merge upstream/master          # conflicts, if any, will be in the files listed above
git submodule update --init --recursive SDK CAPI   # upstream bumps these
cmake --build build-linux-x86_64 -j"$(nproc)"
cd ../../gamemode && ./scripts/dev_build.sh && (cd build/linux-tests && ctest)
git -C ../third_party/open.mp-server push origin serve-m
git -C .. add third_party/open.mp-server && git -C .. commit   # bump the gitlink
```

Resolving a conflict: section A means upstream's line is wrong and ours should win —
unless upstream fixed it themselves, in which case take theirs and delete the row above.
Section B means both are right and the two intents have to be combined by hand.

Never resolve an NPC conflict by taking one side wholesale without reading section B. The
two lines that were reconciled in `5cb485cd` diverged precisely because that felt easier
than merging at the time.

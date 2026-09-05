# SERVE-M fork of open.mp

This is `chavis-mtech/open.mp`, branch `serve-m`. It exists so SERVE-M can ship NPC
behaviour that upstream does not have yet, **without falling behind upstream**. Staying
mergeable matters more than any individual change here: a fork that drifts stops receiving
fixes, and the point of this one is to keep receiving them.

`upstream` = `openmultiplayer/open.mp`. Never push there.

## The whole delta

Nine production-code files. Everything else in the product code is byte-identical to
upstream, and it should stay that way — if a change can be made in SERVE-M's own C++
instead of here, make it there. This document and the two branch-safety hooks are the
only fork-maintenance files.

```
CMakeLists.txt                        +3    clang-cl /EHsc
Server/Source/CMakeLists.txt          +3    macOS: dl without libatomic
Server/Components/NPCs/NPC/npc.cpp    heavy the driving, sync and damage work below
Server/Components/NPCs/NPC/npc.hpp    ~27   fork declarations and state
Server/Components/NPCs/Node/node.cpp  ~184  directional / lane-aware traversal and point-specific z
Server/Components/NPCs/Node/node.hpp  ~19   node traversal and z helpers
Server/Components/NPCs/npcs_impl.cpp  ~59   damage guards, stream-in reseat and carjack hold wiring
Server/Components/NPCs/npcs_impl.hpp  ~8    stream and vehicle event registration
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
| `NPC::processDamage()` | applies damage to a dead NPC and never kills on the lethal hit; death is left to the next tick, which only fires while the player state is on foot. Here a corpse is rejected and the lethal hit kills immediately, with the killer and weapon that actually landed it. |
| `NPC::shoot()` | decides whether a bullet lands from the **shooter's** `dead_`/`invulnerable_`, not the target's. Bullets pass into corpses and into NPCs a script made invulnerable, while an invulnerable shooter cannot hurt anyone (upstream [#1244](https://github.com/openmultiplayer/open.mp/issues/1244)). |
| `NPCComponent::onPlayerGiveDamage()` | raises `onNPCTakeDamage` and subtracts health for a dead NPC, so shots fired into a body read to a script as an NPC alive on 0 HP — the second half of [#1244](https://github.com/openmultiplayer/open.mp/issues/1244). Wasted players do not take damage; nor should NPCs. |
| `NPCNode::getPosition()` | adds the ped sync origin (+1.2, mid-torso) to EVERY path-node z, including vehicle points. A vehicle spawned or driver-synced from a vehicle node hangs ~0.7m above the road — and clients keep unoccupied-vehicle physics asleep until something touches the car, so a parked or abandoned one floats indefinitely. Here vehicle points (which precede ped points in the node file) carry the chassis rest height (+0.5) instead. |
| macOS link | links `atomic` alongside `dl`; libc++/compiler-rt provide atomics and no separate libatomic exists there. |

The first half of #1244 — `NPC_Respawn()` leaving `dead_` set, so `kill()` returns early
forever and `OnNPCDeath` never fires again — is upstream's own bug, fixed by `0dbf39dd`
and present in every build from 3097 onwards. The report was filed against 3079.

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
- **Stream-in reseat** — `NPC::onObserverStreamedIn()`, `NPCComponent::onPlayerStreamIn()`.
  Stream-in (RPC 32) carries no vehicle/seat, so a new observer renders a seated NPC as an
  on-foot ped standing at the vehicle's coordinates until an in-vehicle sync arrives — and
  a parked NPC's change-detector can withhold that packet for the whole skip-update window
  (~sync rate × skip limit). Observers saw stationary drivers physics-pushed around their
  own car, and the native carjack never armed because the seat looked empty locally. On
  stream-in of a seated NPC the skip allowance is exhausted so the next sync slot emits
  unconditionally.
- **Damage animations** — `processDamage()`, `kill()`, `getAnimation()`, `utils.hpp`,
  `npcs_impl.cpp`. NPCs react to being hit instead of absorbing bullets impassively.
- **Carjack drag protection** — `NPCComponent::onPlayerEnterVehicle()`,
  `NPC::holdPeriodicInVehicleSync()`, and the skip-limit branches of
  `sendDriverSync()`/`sendPassengerSync()`. While a human's announced entry task runs
  against a matching NPC-occupied driver or passenger seat, the periodic re-assert of the
  unchanged seated pose is held (~5.8s, matching the jack allowance in onTick) so the
  native drag on the jacker's screen is not interrupted by the victim snapping back into
  the seat. Changed state still syncs immediately; a cancelled entry lets the hold lapse.

## B2. Lane centring and link height (`navigation_math.hpp`)

Two pieces of driving arithmetic moved out of `npc.cpp` into a header with no dependencies, so
they could be tested (from SERVE-M's `platform_policy_tests`, which includes this directory).
Both were wrong in a way only a client shows, and a client finally showed it.

- **Lane offset.** The old formula, `width + 3.5 * (0.5 + lane)`, read the navi node's
  width byte as a median and treated every road as two-way. Checked against all 64
  `NODES*.DAT` files the game ships: the width byte is zero on 97% of nodes and is a WIDTH
  where it is not; ~19,000 navi nodes are two-way `(1,1)` and ~12,000 are one-way with
  lanes on one side only. On a one-way carriageway the navi node is the carriageway's own
  centre, so the outer lane of a two-lane freeway landed 1.75 m past the edge - on the
  retaining wall, as photographed. One-way roads are now centred on the node; a stated
  width bounds every lane inside the road.
- **Height along a link.** `advance()` steered z toward the target at a rate clamped to
  ±35% of horizontal speed. Any ramp steeper than that had the car climb slower than the
  road and drive into it, then snap up at the node; downhill it floated. Height is now read
  off the link by horizontal progress (`moveStartPosition_` → `targetPosition_`), which is
  what a straight piece of road between two dense nodes actually is.

## B3. Per-model rest height (`vehicle_rest_height.hpp`, generated)

Every drive target reaches the NPC at road + 0.5 - the node point's chassis offset. A vehicle's
origin is not 0.5 m above its tyres: a saloon's is ~0.7, an SUV's ~1.0, a Linerunner's 1.5, a
Roadtrain's 2.0. With one constant, saloons drove 0.2 m into the road and trucks a metre or
more. The table is generated by SERVE-M's `tools/vehicle_specs/generate.py` from open.mp's own
vehicle model table (front wheel hub height) and the game's `vehicles.ide` (wheel scale, the
tyre diameter in metres): `rest = -hubFrontZ + wheelScaleFront / 2`. `advance()` lifts the
target by `rest - 0.5` and measures arrival against the lifted target, so a tall truck does
not stay a metre "away" from every node forever.

## C. Considered and deliberately not taken

From the earlier `serve-m` line (`8860acca`), reconciled in `49f19f64`. Recorded because
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
git submodule update --init --recursive            # upstream can bump any nested dependency
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

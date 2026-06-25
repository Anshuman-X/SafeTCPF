# Debug Report — SafeTCPF SUMO Simulation Fix

**Date:** 2026-06-25  
**Status:** ✅ RESOLVED — All vehicles spawn and traverse the intersection correctly.

---

## Errors Found and Fixed

### Bug 1 (CRITICAL): No `<vehicle>` elements in route file

**File:** `sumo_files/net.rou.xml`  
**Root Cause:** The route file contained only `<vType>` definitions and a single `<route>` template (`r_dummy`), but zero `<vehicle>` elements. SUMO had nothing to spawn.  
**Fix:** Added 16 concrete `<vehicle>` definitions covering all 6 route directions (N→S, S→N, E→W, W→E, N→W, S→E) with staggered departure times.  
**Reason:** SUMO's standalone mode requires explicit `<vehicle>` elements in the route file. The TraCI demo spawns via `traci.vehicle.add()`, but the standalone `sumo-gui` config depends on the route file.

### Bug 2 (CRITICAL): Unsorted vehicle depart times

**File:** `sumo_files/net.rou.xml`  
**Root Cause:** Vehicles were grouped by route direction, not sorted by `depart` time. SUMO loads route files incrementally and requires monotonically increasing depart values.  
**Fix:** Reordered all 16 vehicles by depart time: 0, 0, 0, 0, 2, 2, 5, 10, 10, 10, 10, 15, 15, 20, 20, 20.  
**Reason:** SUMO's incremental loader warns and ignores vehicles with a depart time earlier than the previous vehicle's depart time.

### Bug 3 (CRITICAL): `grid_to_sumo()` coordinate mapping was wrong

**File:** `simulation/sumo_env.py`, method `grid_to_sumo()`  
**Root Cause:** The function subtracted `(9.5, 11.5)` and applied a 3.2m/5.0m scaling, producing coordinates in the range `(-47.5, -57.5)` to `(47.5, 62.5)`. The SUMO network uses `netOffset=(100,100)`, so valid world coordinates span `(0, 0)` to `(200, 200)`. All mapped coordinates fell outside any lane.  
**Fix:** Rewrote with lookup tables for exact lane lateral positions (from `net.net.xml` lane shapes: 95.20, 98.40, 101.60, 104.80) and linear interpolation for the longitudinal axis (`y * 200/23` for vertical, `x * 200/19` for horizontal).  
**Reason:** `moveToXY(keepRoute=2)` cannot snap a vehicle to a lane if the coordinates are outside the network bounding box.

### Bug 4 (HIGH): `spawn_pedestrian()` had wrong TraCI API usage

**File:** `simulation/sumo_env.py`, method `spawn_pedestrian()`  
**Root Cause:** Two issues:  
  1. `traci.person.add(ped_id, edge_id, X)` passed a SUMO world coordinate (e.g., 73.68) as the `pos` parameter. `pos` must be the position *along the edge* in meters (0 to edge length ≈ 89.6).  
  2. No walking/waiting stage was appended after `add()`. SUMO requires at least one plan stage or the person is immediately removed.  
**Fix:** Changed to place pedestrian at edge midpoint (`pos=45.0`), append a `waitingStage(1000s)`, then teleport to actual position with `moveToXY(keepRoute=2)`.  
**Reason:** Without a valid position and plan stage, SUMO removes the person before the first simulation step.

### Bug 5 (MEDIUM): `get_edge_id()` boundary conditions

**File:** `simulation/sumo_env.py`, method `get_edge_id()`  
**Root Cause:** Boundary checks used `y >= 12` for N_to_C (should be `y >= 14`) and `y <= 11` for S_to_C (should be `y <= 9`). Grid y=[10..13] is the intersection.  
**Fix:** Corrected thresholds to `y >= 14` (north approach), `y <= 9` (south approach), `x <= 7` (west approach), `x >= 12` (east approach).  
**Reason:** Incorrect edge hints could confuse `moveToXY` when vehicles are near the intersection boundary.

### Bug 6 (LOW): Pedestrian vType width too large

**File:** `sumo_files/net.rou.xml`  
**Root Cause:** `<vType id="ped" width="0.8"/>` exceeded SUMO's default pedestrian stripe width, generating a warning.  
**Fix:** Changed width from `0.8` to `0.5` and length from `0.8` to `0.4`.  
**Reason:** Eliminates the SUMO warning about potential collisions with vehicles.

### Bug 7 (MEDIUM): `generate_xml_files()` overwrites route file

**File:** `simulation/sumo_env.py`, method `generate_xml_files()`  
**Root Cause:** `SumoEnvironment.__init__()` calls `generate_xml_files()` which regenerates `net.rou.xml` from a hardcoded Python string. The original string had no vehicle definitions — any manual fix to the XML file would be overwritten.  
**Fix:** Updated the embedded `routes_content` string to include all 16 vehicle definitions and 7 named routes, matching the standalone `net.rou.xml`.  
**Reason:** Ensures vehicles are present regardless of whether the simulation is started via `sumo-gui` directly or through the Python pipeline.

---

## Files Modified

| File | Changes |
|------|---------|
| `sumo_files/net.rou.xml` | Added 16 vehicles, 7 routes, fixed vType widths, sorted by depart time |
| `simulation/sumo_env.py` | Fixed `grid_to_sumo()`, `get_edge_id()`, `spawn_pedestrian()`, `generate_xml_files()` |

### Bug 8 (HIGH): No sidewalk infrastructure for pedestrians

**Files:** `simulation/sumo_env.py`, `sumo_files/net.ped.xml` (new), `sumo_files/net.sumocfg`  
**Root Cause:** The SUMO network had no sidewalk lanes or pedestrian crossings. SUMO requires dedicated sidewalk lanes (`allow="pedestrian"`) for pedestrians to exist and be visible. Without sidewalks, `traci.person.add()` cannot place a person on any lane, and standalone `<person>` definitions have no valid walking path.  
**Fix:**  
  1. Added `--sidewalks.guess` and `--crossings.guess` flags to `netconvert` — this adds a 2m sidewalk lane (index 0) to every edge and creates pedestrian crossings at the center junction.  
  2. Removed explicit `--connection-files` from `netconvert` — sidewalks shift lane indices (old 0/1 become 1/2), so auto-generated connections are safer.  
  3. Created `net.ped.xml` with 6 `<person>` + `<walk>` definitions crossing the intersection in all directions.  
  4. Updated `net.sumocfg` to load `net.ped.xml` alongside `net.rou.xml`.  
**Reason:** Pedestrians in SUMO can only walk on lanes with `allow="pedestrian"`. The sidewalk lanes and junction crossings provide the necessary infrastructure.

---

## Files Modified

| File | Changes |
|------|---------|
| `sumo_files/net.rou.xml` | Added 16 vehicles, 7 routes, fixed vType widths, sorted by depart time |
| `sumo_files/net.ped.xml` | **NEW** — 6 pedestrian definitions with walk stages |
| `sumo_files/net.sumocfg` | Updated to load both `net.rou.xml` and `net.ped.xml` |
| `simulation/sumo_env.py` | Fixed `grid_to_sumo()`, `get_edge_id()`, `spawn_pedestrian()`, `generate_xml_files()`, `compile_network()` |

---

## Verification Performed

1. **Headless SUMO run:** `sumo -c net.sumocfg --duration-log.statistics --tripinfo-output logs/tripinfo.xml`
   - ✅ 16/16 vehicles inserted and completed
   - ✅ 6/6 pedestrians inserted and completed
   - ✅ 0 teleports
   - ✅ 0 warnings / 0 errors
   - ✅ Vehicle avg route length: 191.96 m, avg duration: 21.44 s
   - ✅ Pedestrian avg route length: 150.21 m, avg duration: 116.33 s
   - ✅ 0 depart delay

2. **Trip verification:** All 16 vehicles and 6 pedestrians have valid arrival times in `logs/tripinfo.xml`, confirming full traversal of the intersection.


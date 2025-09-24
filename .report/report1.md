# 📝 Combined Nav2 & ROS 2 Summary Report

This document summarizes Nav2 global planners, robot size handling, path range generation, collision checks, map vs costmap resolution, and relevant ROS 2 concepts.

---

## 1. Global Planner and Robot Size
- Nav2 global planners (Navfn, Smac, Theta*) **do not check `robot_radius` or `footprint` directly**.  
- Planners query the **costmap**, which incorporates the robot’s size.  
- Robot size models:
  - `robot_radius`: circular approximation
  - `footprint`: polygonal, more accurate
- **Priority**: If both `footprint` and `robot_radius` are set, `footprint` is used; `robot_radius` is ignored.  

### Reference
- Nav2 documentation confirms: `footprint` overrides `robot_radius`.

---

## 2. Path Range Generation
Generate **small, medium, long ranges** (two sets each → six total).  
Ranges scale with **map resolution (meters/pixel)**.  

Example: map resolution = 0.05 m/pixel  

| Range  | Pixels | Meters |
|--------|--------|--------|
| Small  | 20–40  | 1–2    |
| Medium | 60–100 | 3–5    |
| Long   | 120–200| 6–10   |

---

## 3. Collision Checking (Start & End Points)
- Ensure **start/end points are not inside obstacles**.  
- Conversion:
  - Robot radius = 0.65 m → convert to pixels using map resolution.
  - Verify a circle of this radius does **not overlap blocking pixels**.

---

## 4. Map vs Costmap Resolution
- `map.yaml` resolution = native resolution of the static map.  
- Costmap resolution = Nav2’s internal grid representation.  
- **They do not need to match**:
  - Costmap coarser than map → fine.
  - Costmap finer than map → no additional detail.

---

## 5. Footprint and Resolution Relationship
- Footprint is **always in meters**.  
- Costmap resolution determines how many grid cells the footprint covers:

| Robot Radius | Resolution (m/cell) | Cells |
|--------------|-------------------|-------|
| 0.65 m       | 0.05              | ~13   |
| 0.65 m       | 0.1               | ~6–7  |

- Footprint size is **constant in meters**, variable in number of cells.

---

## 6. ROS 2 Parameters Across Nodes
- Parameters are **scoped per node**.  
- To share parameters:
  - Use YAML/launch files.
  - Query with `/get_parameters`.
  - Subscribe to `/parameter_events`.
- Dynamic values → use **topics/services**, not parameters.

---

## 7. ROS 2 Launch Event Handlers
- `on_start` → when process starts  
- `on_exit` → when process exits  
- `on_shutdown` → when launch ends  
- `OnProcessIO` → reacts to stdout/stderr output  
- Lifecycle nodes: `OnStateTransition` triggers on state changes

---

## 8. stdout / stderr
- **stdout**: normal output (INFO, print).  
- **stderr**: errors/warnings.  
- Note: Not ROS topics; just console logs.

---

## 9. LaTeX Snippet Usage
- `tcolorbox` → fancy code boxes  
- `fancyvrb` → preserves raw text  
- `LTR` → force left-to-right mode  

---

## 10. Matplotlib `plt.figure(figsize=(16,12))`
- Creates a plotting canvas **16×12 inches**  
- Pixels = `figsize × dpi` (dpi usually 100)

---

## 11. ROS 2 Run Node with Namespace
```bash
ros2 run my_package my_node --ros-args -r __ns:=/robot1

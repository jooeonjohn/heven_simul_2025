import csv
import math
import os
import copy
import matplotlib.pyplot as plt
from matplotlib.backend_bases import MouseButton
import matplotlib

matplotlib.rcParams['keymap.save'] = []  # disable 's' for save

# Paths
original_csv_path = os.path.expanduser('~/catkin_ws/src/heven_mm_2025/mm_common/csv/main_course.csv')
edited_csv_path = os.path.expanduser('~/catkin_ws/src/heven_mm_2025/mm_common/csv/gps_log_edited.csv')

# Earth model
EARTH_RADIUS_KM = 6378.135
deg_to_m = math.pi * EARTH_RADIUS_KM / 180.0 * 1000.0

# Load original GPS data
latlons = []
with open(original_csv_path, newline='') as csvfile:
    reader = csv.reader(csvfile)
    for row in reader:
        if len(row) != 2:
            continue
        try:
            lat, lon = float(row[0]), float(row[1])
            latlons.append((lat, lon))
        except ValueError:
            continue

if not latlons:
    raise ValueError("No valid GPS points found.")

# Set origin to first point
origin_lat, origin_lon = latlons[0]
cos_lat = math.cos(origin_lat * math.pi / 180)

# Convert lat/lon to relative x/y
xy_points = [
    (
        (lon - origin_lon) * cos_lat * deg_to_m,
        (lat - origin_lat) * deg_to_m
    )
    for lat, lon in latlons
]

# History stacks
undo_stack = []
redo_stack = []

# Plot setup
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([x for x, y in xy_points], [y for x, y in xy_points],
                linestyle='-', marker='o', color='green', picker=True)
ax.set_title("Edit Path — Left drag | Right delete | Middle add\n'z': Undo | 'y': Redo | 's': Save")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)
ax.axis("equal")

dragging_point = None
selected_indices = set()
dragging_selection = False
selection_offsets = {}

def push_undo():
    undo_stack.append(copy.deepcopy(xy_points))
    redo_stack.clear()

def update_plot():
    if xy_points:
        xs, ys = zip(*xy_points)
    else:
        xs, ys = [], []

    line.set_data(xs, ys)

    # Remove old selection markers
    for artist in ax.lines[1:]:
        artist.remove()

    # Draw selected points in red
    for idx in selected_indices:
        x, y = xy_points[idx]
        ax.plot(x, y, 'ro', markersize=8)

    fig.canvas.draw_idle()

def find_nearest_point(x, y, threshold=10.0):
    distances = [(i, (x - px) ** 2 + (y - py) ** 2) for i, (px, py) in enumerate(xy_points)]
    if not distances:
        return None
    idx, dist_sq = min(distances, key=lambda d: d[1])
    return idx if dist_sq < threshold ** 2 else None

def is_toolbar_active():
    return getattr(fig.canvas.manager.toolbar, 'mode', '') != ''

def on_press(event):
    global dragging_point, dragging_selection, selection_offsets
    if event.inaxes != ax or is_toolbar_active():
        return

    x, y = event.xdata, event.ydata
    idx = find_nearest_point(x, y)

    if event.button == MouseButton.LEFT:
        if event.key == 'shift' and idx is not None:
            # Toggle selection
            if idx in selected_indices:
                selected_indices.remove(idx)
            else:
                selected_indices.add(idx)
            update_plot()
        elif idx is not None:
            push_undo()
            if idx in selected_indices:
                # Start dragging all selected
                dragging_selection = True
                selection_offsets = {
                    i: (xy_points[i][0] - x, xy_points[i][1] - y)
                    for i in selected_indices
                }
            else:
                # Start dragging one point
                dragging_point = idx

    elif event.button == MouseButton.RIGHT:
        if idx is not None:
            push_undo()
            xy_points.pop(idx)
            # Adjust selection
            selected_indices.clear()
            update_plot()

    elif event.button == MouseButton.MIDDLE:
        push_undo()
        if not xy_points:
            xy_points.append((x, y))
        else:
            insert_idx = idx + 1 if idx is not None else len(xy_points)
            xy_points.insert(insert_idx, (x, y))
        update_plot()
        
def on_motion(event):
    if event.inaxes != ax or is_toolbar_active():
        return
    x, y = event.xdata, event.ydata

    if dragging_point is not None:
        xy_points[dragging_point] = (x, y)
        update_plot()

    elif dragging_selection:
        for i, (dx, dy) in selection_offsets.items():
            xy_points[i] = (x + dx, y + dy)
        update_plot()

def on_release(event):
    global dragging_point, dragging_selection
    dragging_point = None
    dragging_selection = False

def on_key(event):
    global xy_points, selected_indices

    if event.key == 's':
        with open(edited_csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for x, y in xy_points:
                lat = y / deg_to_m + origin_lat
                lon = x / (cos_lat * deg_to_m) + origin_lon
                writer.writerow([lat, lon])
        print(f"Saved to {edited_csv_path}")
        plt.close()

    elif event.key == 'z':  # Undo
        if undo_stack:
            redo_stack.append(copy.deepcopy(xy_points))
            xy_points[:] = undo_stack.pop()
            selected_indices.clear()
            update_plot()
            print("Undo")

    elif event.key == 'y':  # Redo
        if redo_stack:
            undo_stack.append(copy.deepcopy(xy_points))
            xy_points[:] = redo_stack.pop()
            selected_indices.clear()
            update_plot()
            print("Redo")

    elif event.key == 'escape':  # Clear selection
        selected_indices.clear()
        update_plot()
        print("Selection cleared")

fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)
fig.canvas.mpl_connect('key_press_event', on_key)

plt.show()

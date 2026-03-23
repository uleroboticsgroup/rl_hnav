#!/usr/bin/env python3
"""Capture Gazebo link poses for ankle/foot links over time to detect drift."""
import subprocess, re, time, json

LINKS = ['left_ankle_roll_link', 'right_ankle_roll_link', 
         'left_ankle_pitch_link', 'right_ankle_pitch_link',
         'pelvis']

def capture_poses():
    """Capture one snapshot from gz topic"""
    result = subprocess.run(
        ['docker', 'exec', 'rl_hnav_container', 'bash', '-c',
         'timeout 2 gz topic -e /gazebo/default/pose/info 2>/dev/null'],
        capture_output=True, text=True, timeout=10
    )
    output = result.stdout
    
    # Parse protobuf text format for g1 links
    poses = {}
    # Split by 'pose {' blocks
    blocks = re.split(r'pose\s*\{', output)
    for block in blocks:
        name_match = re.search(r'name:\s*"g1::(\w+)"', block)
        if not name_match:
            continue
        name = name_match.group(1)
        if name not in LINKS:
            continue
        
        pos = {}
        for axis in ['x', 'y', 'z']:
            m = re.search(r'position\s*\{[^}]*?' + axis + r':\s*([0-9eE.+-]+)', block, re.DOTALL)
            if m:
                pos[axis] = float(m.group(1))
        
        ori = {}
        # orientation is after position
        ori_block = re.search(r'orientation\s*\{([^}]*)\}', block)
        if ori_block:
            for axis in ['x', 'y', 'z', 'w']:
                m = re.search(axis + r':\s*([0-9eE.+-]+)', ori_block.group(1))
                if m:
                    ori[axis] = float(m.group(1))
        
        if name not in poses:  # first occurrence
            poses[name] = {'pos': pos, 'ori': ori}
    
    return poses

print("Capturing 5 snapshots, 3s apart...")
snapshots = []
for i in range(5):
    poses = capture_poses()
    snapshots.append(poses)
    print(f"\n=== Snapshot {i+1} (t={i*3}s) ===")
    for link in LINKS:
        if link in poses:
            p = poses[link]['pos']
            o = poses[link]['ori']
            print(f"  {link:30s}  pos=({p.get('x',0):.4f}, {p.get('y',0):.4f}, {p.get('z',0):.4f})"
                  f"  ori=({o.get('x',0):.5f}, {o.get('y',0):.5f}, {o.get('z',0):.5f}, {o.get('w',0):.5f})")
        else:
            print(f"  {link:30s}  NOT FOUND")
    if i < 4:
        time.sleep(3)

# Compute drift between first and last
if len(snapshots) >= 2 and snapshots[0] and snapshots[-1]:
    print("\n=== DRIFT (snapshot 5 - snapshot 1) ===")
    for link in LINKS:
        if link in snapshots[0] and link in snapshots[-1]:
            p0 = snapshots[0][link]['pos']
            p1 = snapshots[-1][link]['pos']
            o0 = snapshots[0][link]['ori']
            o1 = snapshots[-1][link]['ori']
            dp = {k: p1.get(k,0) - p0.get(k,0) for k in ['x','y','z']}
            do = {k: o1.get(k,0) - o0.get(k,0) for k in ['x','y','z','w']}
            print(f"  {link:30s}  dpos=({dp['x']:.6f}, {dp['y']:.6f}, {dp['z']:.6f})"
                  f"  dori=({do['x']:.6f}, {do['y']:.6f}, {do['z']:.6f}, {do['w']:.6f})")

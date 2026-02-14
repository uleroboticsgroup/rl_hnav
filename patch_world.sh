#!/bin/bash

set -e

echo "=== Gazebo World Patch Script ==="

W_ORIG="$HOME/rl_hnav/install/g1_gazebo/share/g1_gazebo/worlds/turtlebot3_house.world"
W_PATCH="$HOME/rl_hnav/install/g1_gazebo/share/g1_gazebo/worlds/turtlebot3_house_patched.world"

if [ ! -f "$W_ORIG" ]; then
  echo "ERROR: Original world not found:"
  echo "$W_ORIG"
  exit 1
fi

echo "1) Copying original world..."
cp -f "$W_ORIG" "$W_PATCH"

echo "2) Removing mailbox include blocks..."
perl -0777 -i -pe 's/<include>.*?<uri>\s*model:\/\/mailbox\s*<\/uri>.*?<\/include>\s*//sg' "$W_PATCH"

echo "3) Removing first_2015_trash_can include blocks..."
perl -0777 -i -pe 's/<include>.*?<uri>\s*model:\/\/first_2015_trash_can\s*<\/uri>.*?<\/include>\s*//sg' "$W_PATCH"

echo "4) Checking result..."
if grep -q "model://mailbox\|model://first_2015_trash_can" "$W_PATCH"; then
  echo "WARNING: Some references still exist!"
else
  echo "OK: no missing models referenced"
fi

echo ""
echo "✅ PATCHED WORLD CREATED:"
echo "$W_PATCH"
